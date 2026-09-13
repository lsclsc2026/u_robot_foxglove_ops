#!/usr/bin/env python3
"""任务协调器主节点（第一步最简版）"""

import importlib.util
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import String

from mos_coordinator.states import Phase, Step, TaskState
from mos_coordinator.adapters import NavigationAdapter, VisionAdapter, ArmAdapter, FetchAdapter


def _import_load_nav():
    """导入 config/load_nav.py（源码目录或 share 安装目录）。"""
    mod_name = 'mos_coordinator_load_nav'
    if mod_name in sys.modules:
        return sys.modules[mod_name]

    candidates = [
        os.path.normpath(
            os.path.join(os.path.dirname(__file__), '..', 'config', 'load_nav.py')
        ),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            os.path.join(
                get_package_share_directory('mos_coordinator'),
                'config',
                'load_nav.py',
            )
        )
    except Exception:
        pass

    for path in candidates:
        if os.path.isfile(path):
            spec = importlib.util.spec_from_file_location(mod_name, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[mod_name] = module
            spec.loader.exec_module(module)
            return module
    raise ImportError('无法找到 config/load_nav.py')


load_nav = _import_load_nav()


class TaskCoordinator(Node):
    
    def __init__(self):
        super().__init__('task_coordinator')

        # ===== 初始化子系统适配器 =====
        self.navigation = NavigationAdapter(self)
        self.vision = VisionAdapter(self)
        self.arm = ArmAdapter(self)
        self.fetcher = FetchAdapter(
            node=self,
            vision=self.vision,
            navigation=self.navigation,
            arm=self.arm,
        )

        # ===== 两层状态：Phase（业务） + Step（子步骤） =====
        self.phase = Phase.IDLE
        self.step = Step.Cruise
        self.previous_phase = None
        self.previous_step = None
        # Act 内部细状态（接近/对准/抓取等），仅 Step.Act 时有意义
        self.act_detail = None
        self.last_error_msg = ""
        self.previous_state = None  # 兼容旧错误重试逻辑
        self.state_retry_count = {}
        self.max_retry_per_state = 3
        
        # ===== 位置信息存储 =====
        # 默认位姿（避免空指针）
        self.current_pose_symbol = None
        #self.current_pose = PoseStamped()

        
        self.declare_parameter('robot_id', 'R1')
        self.robot_id = self.get_parameter('robot_id').value

        # 从 config/coordinator_params.yaml 读点，自动填充导航与符号表
        x = load_nav.apply_to_node(self)
        self.get_logger().info(f'x: {x}, self.navigation_yaml: {self.navigation_yaml}')
        
        
        # ===== 发布状态信息 =====
        self.status_pub = self.create_publisher(String, '/coordinator/status', 10)
        status_rate = 1.0  # 默认发布频率
        self.create_timer(1.0 / status_rate, self.publish_status)

        # ===== 巡航和检测状态 =====
        self.is_cruising = False  # 是否正在巡航
        self.current_target_symbol = None  # 当前目标点符号（A/B/D/E）
        self.is_holding_cart = False  # 是否手持小车
        self.fullness_check_active = False  # 装载检测是否激活

        # ===== 接近目标相关 =====
        self.current_cart_pose = None  # 当前检测到的小车位置 (geometry_msgs/Point)
        self.current_cart_class = None  # 小车类别 ID
        self.approach_retry_count = 0  # 接近重试次数
        self.max_approach_retries = 3  # 最大接近重试次数

        # ===== 把手检测和抓取相关 =====
        self.last_handle_left = None  # 最后一次检测到的左手把手位置
        self.last_handle_right = None  # 最后一次检测到的右手把手位置
        self.grasp_retry_count = 0  # 抓取重试次数
        self.max_grasp_retries = 3  # 最大抓取重试次数
        self.alignment_threshold = 0.85  # 对准阈值（米）- 把手距离超过此值需要对准

        # ===== 运输和放置相关 =====
        self.transport_target_pose = None  # 运输目标位置（B点）
        self.release_retry_count = 0  # 释放重试次数
        self.max_release_retries = 3  # 最大释放重试次数

        # ===== 状态机主循环定时器 =====
        self.create_timer(1, self.state_machine_loop)  # 每0.5秒执行一次状态机
        
        # ===== 状态跳转订阅（调试用，暂时禁用）=====
        # self.state_jump_sub = self.create_subscription(
        #     String, '/coordinator/jump_to_state', self._jump_to_state, 10
        # )

        # ===== 搜索上下文（SearchFull 等）=====
        self.cruising = False
        self.observing = False
        self.acting = False

        self.current_pos_symbol = None
        self.target_pos_symbol = None
        self.target_cart_pos = None

        self.get_logger().info("=" * 50)
        self.get_logger().info("任务协调器已启动")
        self.get_logger().info(
            f"初始状态: {self.phase.value}/{self.step.value}"
        )
        self.get_logger().info("=" * 50)

    @property
    def state(self):
        """兼容旧字段：state 即当前 Phase。"""
        return self.phase

    @state.setter
    def state(self, value):
        self.phase = value

    # ===== Adapter事件转发到FetchAdapter =====
    
    def on_handle_positions_received(
        self, 
        left_handle: PointStamped, 
        right_handle: PointStamped, 
        task_name: str = 'grasp'
    ):
        """VisionAdapter → Coordinator → FetchAdapter"""
        if hasattr(self, 'fetcher') and self.fetcher is not None:
            self.fetcher.notify_handle_positions(
                left_handle, right_handle, task_name
            )
    
    def on_alignment_complete(self, success: bool = True):
        """NavigationAdapter → Coordinator → FetchAdapter"""
        if hasattr(self, 'fetcher') and self.fetcher is not None:
            self.fetcher.notify_alignment_complete(success)
    
    def on_grasp_result(self, success: bool):
        """ArmAdapter → Coordinator → FetchAdapter"""
        if hasattr(self, 'fetcher') and self.fetcher is not None:
            self.fetcher.notify_grasp_result(success)


    def transition_phase(
        self,
        new_phase: Phase,
        error_msg: str = "",
        reset_step: bool = True,
    ):
        """切换整体业务阶段。"""
        self.previous_phase = self.phase
        self.phase = new_phase
        if reset_step:
            self.previous_step = self.step
            self.step = Step.Cruise
            self.act_detail = None
        self.get_logger().info(
            f"Phase: {self.previous_phase.value} -> {self.phase.value}"
            f" (step={self.step.value})"
        )
        if new_phase == Phase.ERROR and error_msg:
            self.get_logger().error(f"错误: {error_msg}")
            self.last_error_msg = error_msg

    def transition_step(self, new_step: Step):
        """切换当前 Phase 内的子步骤。"""
        self.previous_step = self.step
        self.step = new_step
        self.get_logger().info(
            f"Step: {self.previous_step.value} -> {self.step.value}"
            f" [{self.phase.value}]"
        )

    def get_cart_pos_by_symbol(self, symbol: str = None):
        """根据当前搜索车位计算目标抓车位（如 A → ACart）。"""
        if symbol is None:
            symbol = self.current_pose_symbol
        return self.CartSymbols.get(symbol)

    def _create_pose(self, x, y, yaw):
        """创建 PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def publish_status(self):
        """定期发布当前两层状态：Phase/Step。"""
        msg = String()
        detail = f"/{self.act_detail}" if self.act_detail else ""
        msg.data = f"{self.phase.value}/{self.step.value}{detail}"
        self.status_pub.publish(msg)

    def state_machine_loop(self):
        """状态机主循环：先处理 Step.GoBack，再按 Phase 分发。"""
        if self.phase == Phase.IDLE:
            return
        if self.phase == Phase.ERROR:
            # 未来补充错误处理, 如回到初始位置重新开始, 如进行人工呼叫
            return

        if self.phase == Phase.SearchFull:
            self._run_search_full()
        elif self.phase == Phase.PutFull:
            self._run_put_full()
        elif self.phase == Phase.SearchEmpty:
            self._run_search_empty()
        elif self.phase == Phase.PutEmpty:
            self._run_put_empty()
        else:
            self.get_logger().error(f"未知状态: {self.phase.value}")
            return

    # ===== 原有的事件驱动方法 =====

    def search_for_empty_cart(self):
        """找空阶段：在DE区间巡航并检测空载小车

        流程：
        1. 启动DE巡航（D -> E -> D -> E...）
        2. 持续检测装载状态
        3. 检测到空车时，停止巡航并准备抓取
        """
        if not self.is_cruising:
            # 首次进入，启动DE巡航
            self.get_logger().info("=" * 50)
            self.get_logger().info("【找空阶段】启动DE区间巡航")
            self.get_logger().info("=" * 50)

            # 设置巡航路径为DE
            self.navigation.set_patrol_route(['D', 'E'], self.patrol_points_DE)

            # 启动巡航
            self.navigation.start_patrol()
            self.is_cruising = True

            # 启动装载检测（检测空车）
            self.vision.request_fullness_check("left")
            self.vision.request_fullness_check("right")
            self.fullness_check_active = True

            self.get_logger().info("✅ 巡航已启动，装载检测已激活")
            self.get_logger().info("等待检测空载小车...")
        else:
            # 巡航进行中，等待装载检测结果
            pass

    def _run_search_empty(self):
        """SearchEmptyCart：Cruise → Observe → Act → Done。"""
        if self.step == Step.Cruise:
            self._cruise()
        elif self.step == Step.Observe:
            self._search_observe()
        elif self.step == Step.Act:
            self._act()
        elif self.step == Step.Done:
            self.transition_phase(Phase.PutEmpty)
            self.target_pos_symbol = self.SYMBOL_LST[self.robot_id][self.phase.value][0]

    def _run_put_empty(self):
        """PutEmpty：放空车"""
        if self.step == Step.Cruise:
            self.get_logger().info("【放空】巡航")
            self._cruise()
        elif self.step == Step.Observe:
            self.get_logger().info("【放空】观察")
            self._search_observe()
        elif self.step == Step.Act:
            self.get_logger().info("【放空】动作")
            self._act()
        elif self.step == Step.Done:
            self.get_logger().info("【放空】完成")
            self.transition_phase(Phase.SearchFull)
            self.target_pos_symbol = self.SYMBOL_LST[self.robot_id][self.phase.value][0]

    def _run_put_full(self):
        """PutFull：放满车"""
        if self.step == Step.Cruise:
            self.get_logger().info("【放满】巡航")
            self._cruise()
        elif self.step == Step.Observe:
            self._search_observe()
        elif self.step == Step.Act:
            self._act()
        elif self.step == Step.Done:
            self.transition_phase(Phase.SearchEmpty)
            self.target_pos_symbol = self.SYMBOL_LST[self.robot_id][self.phase.value][0]

    def _run_search_full(self):
        """SearchFull：Cruise → Observe → Act → Done。"""
        if self.step == Step.Cruise:
            self._cruise()
        elif self.step == Step.Observe:
            self._search_observe()
        elif self.step == Step.Act:
            self._act()
        elif self.step == Step.Done:
            self.transition_phase(Phase.PutFull)
            self.target_pos_symbol = self.SYMBOL_LST[self.robot_id][self.phase.value][0]

    def get_next_pos(self, current_pos, phase=None):
        """SearchFull 区间内的下一个搜索点。"""
        self.get_logger().info(f'获取下一个搜索点: current_pos: {current_pos}, phase: {self.phase.value}, search_list: {self.SYMBOL_LST[self.robot_id][self.phase.value]}')
        search_list = self.SYMBOL_LST[self.robot_id][self.phase.value]
        if current_pos not in search_list:
            return search_list[0]
        idx = search_list.index(current_pos)
        if idx >= len(search_list) - 1:
            return search_list[0]
        return search_list[idx + 1]

    def arrive_at_cart_pos(self):
        """判断是否到达抓车/放车位置"""
        return self.target_cart_pos \
            and self.current_pose_symbol == self.target_cart_pos \
            and self.current_pose_symbol in self.CartSymbolLst

    def arrive_at_observe_pos(self):
        """判断是否到达观察点"""
        return self.current_pose_symbol in self.SYMBOL_LST[self.robot_id][self.phase.value]

    def get_pose_by_symbol(self, symbol: str):
        """根据符号获取位置"""
        self.get_logger().info(f'获取位置: self.robot_id, {symbol}')
        return self.navigation_yaml[self.robot_id][symbol]

    def _cruise(self):
        """导航中：等待到达后更新位置，再进入 Observe。"""
        if self.is_cruising:
            # ================================================
            # 第一步：检查是否超时（关键优化：取消并重发）
            # ================================================
            if not hasattr(self, 'nav_start_time') or self.nav_start_time is None:
                # 首次进入或重置，记录导航开始时间
                self.nav_start_time = time.time()
                self.get_logger().info(
                    f"🕐 [DEBUG] 导航计时开始: {self.nav_start_time}"
                )

            elapsed = time.time() - self.nav_start_time
            nav_timeout = 90.0  # 90秒超时

            # 每5秒输出一次调试信息
            if int(elapsed) % 5 == 0 and elapsed > 0:
                self.get_logger().info(
                    f"🕐 [DEBUG] 导航进行中: 已用时 {elapsed:.1f}s / {nav_timeout}s"
                )

            if elapsed > nav_timeout:
                # 超时，取消导航并重新发布（关键优化）
                self.get_logger().warn(
                    f"⚠️ [Cruise] 导航超时 ({elapsed:.1f}s)，取消并重发"
                )
                self.get_logger().info(
                    f"[DEBUG] 超时前状态: is_cruising={self.is_cruising}, "
                    f"target={self.target_pos_symbol}"
                )

                self.navigation.cancel_navigation()
                self.is_cruising = False
                self.nav_start_time = None

                self.get_logger().info(
                    f"[DEBUG] 超时后状态: is_cruising={self.is_cruising}, "
                    f"nav_start_time={self.nav_start_time}"
                )
                # 下次循环会重新发布导航命令
                return

            # ================================================
            # 第二步：检查是否到达目标（基于实际位置）
            # ================================================
            if self.current_pose_symbol is None:
                target_pose = self.get_pose_by_symbol(self.target_pos_symbol)
                if self.navigation.current_pose is not None and target_pose is not None:
                    # 计算与目标的距离
                    dx = self.navigation.current_pose.pose.position.x - target_pose[0]
                    dy = self.navigation.current_pose.pose.position.y - target_pose[1]
                    distance = math.sqrt(dx * dx + dy * dy)

                    # 如果距离小于阈值，认为已到达
                    if distance < 0.5:  # 50cm 到达阈值
                        self.get_logger().info(f'✅ 基于定位判断已到达 {self.target_pos_symbol}，距离={distance:.2f}m')
                        self.is_cruising = False
                        self.nav_start_time = None  # 重置时间
                        self.current_pose_symbol = self.target_pos_symbol

                        # 若到达的是抓车位，直接进入 Act
                        if self.arrive_at_cart_pos():
                            time.sleep(10)
                            self.transition_step(Step.Act)
                            return
                        elif self.arrive_at_observe_pos():
                            time.sleep(10)
                            self.transition_step(Step.Observe)
                            return
                        else:
                            self.target_pos_symbol = self.SYMBOL_LST[self.robot_id][self.phase.value][0]
                            self.get_logger().error(f"不在搜索点，去本阶段观察点: {self.target_pos_symbol}")
                            time.sleep(2)
                            self.transition_step(Step.Cruise)
                            return

            # ================================================
            # 第三步：检查 NavigationAdapter 的回调标志
            # ================================================
            if self.navigation.on_goal_reached is None and self.navigation.on_goal_failed is None:
                # 仍在导航中，等待
                pass
            elif self.navigation.on_goal_reached:
                self.navigation.on_goal_reached = None
                self.is_cruising = False
                self.nav_start_time = None  # 重置时间
                self.current_pose_symbol = self.target_pos_symbol
                self.get_logger().info(
                    f'[{self.phase.value}/Cruise] 到达 {self.current_pose_symbol}'
                )
                # 若到达的是抓车位，直接进入 Act
                if self.arrive_at_cart_pos():
                    time.sleep(10)
                    self.transition_step(Step.Act)
                    return
                elif self.arrive_at_observe_pos():
                    time.sleep(10)
                    self.transition_step(Step.Observe)
                    return
                else:
                    self.target_pos_symbol = self.SYMBOL_LST[self.robot_id][self.phase.value][0]
                    self.get_logger().error(f"不在搜索点，去本阶段观察点: {self.target_pos_symbol}")
                    time.sleep(10)
                    self.transition_step(Step.Cruise)
                    return
            elif self.navigation.on_goal_failed:
                self.navigation.on_goal_failed = None
                self.is_cruising = False
                self.nav_start_time = None  # 重置时间
                self.get_logger().error("❌ 导航失败, 继续巡航")
                time.sleep(2)
                self.transition_step(Step.Cruise)
                return
            return
        else:
            # ================================================
            # 不在导航中：发送新的导航命令
            # ================================================
            self.is_cruising = True
            self.nav_start_time = time.time()  # 记录开始时间
            self.get_logger().info(f'[hiCruise] 导航到: {self.target_pos_symbol}')
            x, y, yaw, map_frame = self.get_pose_by_symbol(self.target_pos_symbol)
            self.get_logger().info(f'[Cruise] 导航到: {self.target_pos_symbol} ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°, {map_frame})')
            navigation_rslt = self.navigation.navigate_to_pose(x, y, yaw, map_frame)
            if navigation_rslt:
                self.is_cruising = True
            else:
                self.is_cruising = False
                self.nav_start_time = None  # 重置时间
                self.get_logger().error("❌ 导航失败, 继续巡航")
                time.sleep(2)
                self.transition_step(Step.Cruise)
                return

        # 尚无导航目标：先去当前搜索点（或保持并观察）
        if self.current_pose_symbol is None:
            self.current_pose_symbol = self.SYMBOL_LST[self.robot_id][self.phase.value][0]
        self.transition_step(Step.Cruise)

    def target_found_by_vision(self, phase_value: str, decided_cart_status_and_rslt_flag: tuple):
        _cart_result, is_full = decided_cart_status_and_rslt_flag
        if _cart_result == 'no_cart' and is_full:
            return False
        if phase_value == Phase.SearchFull.value:
            return is_full
        elif phase_value == Phase.PutFull.value:
            return _cart_result == "no_cart"
        elif phase_value == Phase.SearchEmpty.value:
            return _cart_result == "empty"
        elif phase_value == Phase.PutEmpty.value:
            return _cart_result == "no_cart"
        else:
            self.get_logger().error(f"未知阶段: {phase_value}")
            return False

    def target_found(self, phase_value: str):
        """目标找到"""
        self.vision.observe()
        import random
        return random.randint(0, 10) < 9
        cart_result = self.vision.cart_result()
        if phase_value == Phase.SearchFull.value:
            return cart_result == "full"
        elif phase_value == Phase.PutFull.value:
            return cart_result == "not_full"
        elif phase_value == Phase.PutEmpty.value:
            return cart_result == "empty"
        else:
            return cart_result == "no_cart"

    def _search_observe(self):
        """观察：开视觉，判满载；满则去抓车位，否则去下一点。"""
        self.get_logger().info(f'[Observe] 观察在: {self.current_pose_symbol}, phase={self.phase.value}, step={self.step.value}')
        if self.observing:
            if self.vision.decided_cart_status_and_rslt_flag is not None:
                self.count_observe += 1
                if self.target_found_by_vision(self.phase.value, self.vision.decided_cart_status_and_rslt_flag):
                    self.vision.decided_cart_status_and_rslt_flag = None
                    self.target_cart_pos = self.get_cart_pos_by_symbol(self.current_pose_symbol)
                    self.target_pos_symbol = self.target_cart_pos
                    time.sleep(10)
                    self.get_logger().info(
                        f'[Observe] target_found, current={self.current_pose_symbol}, '
                        f'target_cart={self.target_cart_pos}'
                    )
                    self.get_logger().info(f'[{self.phase.value}/{self.current_pos_symbol}/Observe] target_found: {self.target_cart_pos}, self.vision.decided_cart_status_and_rslt_flag: {self.vision.decided_cart_status_and_rslt_flag}')
                    self.observing = False
                    self.get_logger().info(f'目标车位找到 → Cruise {self.target_cart_pos}')
                    self.transition_step(Step.Cruise)
                else:
                    self.vision.decided_cart_status_and_rslt_flag = None
                    if self.count_observe < 10:
                        self.observing = False
                        time.sleep(10)
                        self.get_logger().info(f'[{self.phase.value}/{self.current_pos_symbol}/Observe] 观察开始 {self.count_observe} 次')
                        return
                    next_pos_symbol = self.get_next_pos(self.current_pose_symbol)
                    self.target_pos_symbol = next_pos_symbol
                    self.get_logger().info(f'无目标车位 → 下一搜索点 {self.target_pos_symbol}')
                    self.observing = False
                    self.transition_step(Step.Cruise)
                    return
        else:
            self.observing = True
            self.count_observe = 0
            time.sleep(10)
            self.get_logger().info(f'[{self.phase.value}/{self.current_pos_symbol}/Observe] 观察开始')
            self.vision.observe()

    def get_action_by_phase(self, phase: str):
        self.get_logger().info(f'获取动作: {phase}')
        """根据阶段获取动作"""
        if phase == str(Phase.SearchFull.value):
            return 'fetch'
        elif phase == str(Phase.PutFull.value):
            return 'put'
        elif phase == str(Phase.SearchEmpty.value):
            return 'fetch'
        elif phase == str(Phase.PutEmpty.value):
            return 'put'
        else:
            return None

    def _act(self):
        """动作：在抓车位执行 fetch/放下。"""
        action = self.get_action_by_phase(self.phase.value)
        self.get_logger().info(
            f'[{self.phase.value}/Act] {action} at {self.current_pose_symbol}'
        )
        
        if action == 'fetch':
            # ✅ 异步fetch模式：启动 → 等待 → 获取结果
            if not self.acting:
                # 首次进入：启动异步fetch
                self.acting = True
                self.fetcher.fetch_async()
                return  # 立即返回，等待下次state_machine_loop检查
            
            # 后续循环：检查fetch状态
            if self.fetcher.is_fetch_running():
                # 仍在运行中，继续等待
                return
            
            # fetch完成，获取结果
            ok = self.fetcher.get_fetch_result()
            self.acting = False
            
            if ok:
                self.get_logger().info('Fetch 成功')
                self.transition_step(Step.Done)
            else:
                self.get_logger().error('Fetch 失败 → 继续巡航')
                self.target_pos_symbol = self.get_next_pos(self.current_pose_symbol)
                self.transition_step(Step.Cruise)
        
        elif action == 'put':
            # ✅ 异步 put 模式：启动 → 等待 → 获取结果
            if not self.acting:
                # 首次进入：启动异步 put
                self.acting = True
                self.arm.put_async()
                return  # 立即返回，等待下次 state_machine_loop 检查

            # 后续循环：检查 put 状态
            if self.arm.is_put_running():
                # 仍在运行中，继续等待
                return

            # put 完成，获取结果
            ok = self.arm.get_put_result()
            self.acting = False

            if ok:
                self.get_logger().info('Put 成功')
                self.transition_step(Step.Done)
            else:
                self.get_logger().error('Put 失败 → 继续巡航')
                self.target_pos_symbol = self.get_next_pos(self.current_pose_symbol)
                self.transition_step(Step.Cruise)

    def start_task(self):
        """启动任务。"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("任务开始")
        self.get_logger().info("=" * 50)
        self.current_pose_symbol = self.SYMBOL_LST[self.robot_id][Phase.SearchFull.value][0]
        self.target_pos_symbol = self.current_pose_symbol
        self.transition_phase(Phase.SearchFull)
        self.transition_step(Step.Cruise)

    def stop_task(self):
        """停止任务。"""
        self.get_logger().info("停止任务")
        if hasattr(self.navigation, 'stop_patrol'):
            self.navigation.stop_patrol()
        self.transition_phase(Phase.IDLE)

    def emergency_stop(self):
        """紧急停止"""
        self.get_logger().error("紧急停止！")
        self.stop_task()
        #self.state = TaskState.ERROR
        self.transition_phase(Phase.ERROR, "紧急停止")

def main(args=None):
    rclpy.init(args=args)

    coordinator = TaskCoordinator()

    try:
        coordinator.start_task()
        rclpy.spin(coordinator)

    except KeyboardInterrupt:
        coordinator.get_logger().info("收到中断信号")

    finally:
        coordinator.stop_task()
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

