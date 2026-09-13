#!/usr/bin/env python3
"""完整的 Dummy 测试：模拟所有依赖，从头运行 Coordinator 状态机"""

import sys
import time
import random

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class Phase:
    """整体业务阶段（枚举模拟）"""
    IDLE = "IDLE"
    SearchFull = "SearchFull"
    PutFull = "PutFull"
    SearchEmpty = "SearchEmpty"
    PutEmpty = "PutEmpty"
    ERROR = "ERROR"


class Step:
    """业务阶段内的子步骤（枚举模拟）"""
    Cruise = "Cruise"
    Observe = "Observe"
    Act = "Act"
    Done = "Done"
    GoBack = "GoBack"


class DummyNavigation:
    """模拟导航系统"""
    
    def __init__(self, logger=None):
        self.logger = logger or print
        self._current_pos = None
        self._on_goal_reached = None
    
    @property
    def on_goal_reached(self):
        return self._on_goal_reached
    
    @on_goal_reached.setter
    def on_goal_reached(self, callback):
        self._on_goal_reached = callback
    
    def navigate_to_pose_async(self, x, y, yaw, frame_id='map'):
        """异步导航到目标位置"""
        self.logger(f"📍 导航到: ({x:.2f}, {y:.2f}), yaw={yaw:.1f}°")
        self._current_pos = (x, y)
        # 模拟导航成功
        if self._on_goal_reached:
            self._on_goal_reached()
        return True


class DummyVision:
    """模拟视觉系统"""
    
    def __init__(self, logger=None):
        self.logger = logger or print
        self._enabled = False
    
    def set_vision_enabled(self, enabled):
        self._enabled = enabled
        self.logger(f"👁️ 视觉系统 {'启用' if enabled else '禁用'}")
    
    def cart_result(self):
        """模拟装载检测结果"""
        result = random.randint(0, 9)
        if result < 4:
            self.logger('🎯 检测到满车！')
            return "full"
        elif result < 6:
            self.logger('🎯 检测到非满车！')
            return "not_full"
        elif result < 7:
            self.logger('🎯 检测到空车！')
            return "empty"
        else:
            self.logger('🎯 未检测到车辆！')
            return "no_cart"
    
    def fetch(self):
        """模拟抓取动作"""
        self.logger('🤖 执行抓取...')
        time.sleep(0.5)
        self.logger('✅ 抓取成功！')
        return True


class DummyArm:
    """模拟机械臂系统"""
    
    def __init__(self, logger=None):
        self.logger = logger or print


class DummyCoordinator:
    """模拟 Coordinator 核心逻辑"""
    
    def __init__(self, use_ros2=False):
        self.use_ros2 = use_ros2
        self.logger = self._create_logger()
        
        # 初始化模拟子系统
        self.navigation = DummyNavigation(self.logger)
        self.vision = DummyVision(self.logger)
        self.arm = DummyArm(self.logger)
        
        # 两层状态
        self.phase = Phase.IDLE
        self.step = Step.Cruise
        self.previous_phase = None
        self.previous_step = None
        
        # 机器人配置
        self.robot_id = 'R1'
        self.SYMBOL_LST = {
            'R1': {
                'SearchFull': ['A', 'B'],
                'SearchEmpty': ['D', 'E'],
                'PutFull': ['D', 'E'],
                'PutEmpty': ['A', 'B'],
            },
            'R2': {
                'SearchFull': ['D', 'E'],
                'SearchEmpty': ['F', 'G'],
                'PutFull': ['F', 'G'],
                'PutEmpty': ['D', 'E'],
            }
        }
        
        self.CartSymbols = {
            'A': 'ACart', 'B': 'BCart', 'C': 'CCart',
            'D': 'DCart', 'E': 'ECart'
        }
        
        # 导航坐标配置
        self.navigation_yaml = {
            'R1': {
                'A': (1.0, 1.0, 0.0, 'map'),
                'B': (2.0, 1.0, 0.0, 'map'),
                'D': (3.0, 1.0, 0.0, 'map'),
                'E': (4.0, 1.0, 0.0, 'map'),
            },
            'R2': {
                'D': (3.0, 1.0, 0.0, 'map'),
                'E': (4.0, 1.0, 0.0, 'map'),
                'F': (5.0, 1.0, 0.0, 'map'),
                'G': (6.0, 1.0, 0.0, 'map'),
            }
        }
        
        # 巡航和检测状态
        self.is_cruising = False
        self.current_pos_symbol = None
        self.target_pos_symbol = None
        self.target_cart_pos = None
        
        # 观察和动作状态
        self.observing = False
        self.acting = False
        
        self.logger("=" * 60)
        self.logger("🚀 Dummy Coordinator 已启动")
        self.logger(f"初始状态: {self.phase} / {self.step}")
        self.logger("=" * 60)
    
    def _create_logger(self):
        if self.use_ros2:
            def ros_logger(msg):
                self.node.get_logger().info(msg)
            return ros_logger
        else:
            def simple_logger(msg):
                timestamp = time.strftime('%H:%M:%S', time.localtime())
                print(f"[{timestamp}] {msg}")
            return simple_logger
    
    def transition_phase(self, new_phase, error_msg="", reset_step=True):
        """切换整体业务阶段"""
        self.previous_phase = self.phase
        self.phase = new_phase
        if reset_step:
            self.previous_step = self.step
            self.step = Step.Cruise
        self.logger(f"🔄 Phase: {self.previous_phase} -> {self.phase} (step={self.step})")
        if new_phase == Phase.ERROR and error_msg:
            self.logger(f"❌ 错误: {error_msg}")
    
    def transition_step(self, new_step):
        """切换当前 Phase 内的子步骤"""
        self.previous_step = self.step
        self.step = new_step
        self.logger(f"🔄 Step: {self.previous_step} -> {self.step} [{self.phase}]")
    
    def get_pose_by_symbol(self, symbol):
        """根据符号获取位置"""
        return self.navigation_yaml[self.robot_id].get(symbol, (0.0, 0.0, 0.0, 'map'))
    
    def get_next_pos(self, current_pos):
        """获取下一个搜索点"""
        search_list = self.SYMBOL_LST[self.robot_id].get(self.phase, [])
        if current_pos not in search_list:
            return search_list[0] if search_list else None
        idx = search_list.index(current_pos)
        if idx >= len(search_list) - 1:
            return search_list[0]
        return search_list[idx + 1]
    
    def get_cart_pos_by_symbol(self, symbol=None):
        """根据搜索车位计算目标抓车位"""
        if symbol is None:
            symbol = self.current_pos_symbol
        return self.CartSymbols.get(symbol)
    
    def arrive_at_cart_pos(self):
        """判断是否到达抓车/放车位置"""
        return self.target_cart_pos and \
               self.current_pos_symbol == self.target_cart_pos
    
    def arrive_at_observe_pos(self):
        """判断是否到达观察点"""
        return self.current_pos_symbol in self.SYMBOL_LST[self.robot_id].get(self.phase, [])
    
    def target_found(self, phase_value):
        """模拟目标检测"""
        result = self.vision.cart_result()
        if phase_value == Phase.SearchFull:
            return result == "full"
        elif phase_value == Phase.PutFull:
            return result == "not_full"
        elif phase_value == Phase.PutEmpty:
            return result == "empty"
        else:
            return result == "no_cart"
    
    def _cruise(self):
        """巡航：导航到目标位置"""
        if self.is_cruising:
            return
        
        self.logger(f"🛤️ [Cruise] 开始巡航，当前位置: {self.current_pos_symbol}")
        self.is_cruising = True
        
        if self.target_pos_symbol is None:
            # 尚无目标，设置为当前阶段的第一个点
            phase_points = self.SYMBOL_LST[self.robot_id].get(self.phase, [])
            if phase_points:
                self.target_pos_symbol = phase_points[0]
                self.logger(f"  设定目标点: {self.target_pos_symbol}")
            else:
                self.logger("  ❌ 无可用巡航点")
                self.is_cruising = False
                return
        
        # 执行导航
        x, y, yaw, map_frame = self.get_pose_by_symbol(self.target_pos_symbol)
        self.navigation.navigate_to_pose_async(x, y, yaw, map_frame)
        
        # 模拟导航到达
        time.sleep(0.3)
        self.current_pos_symbol = self.target_pos_symbol
        self.is_cruising = False
        
        self.logger(f"✅ [Cruise] 到达: {self.current_pos_symbol}")
        
        # 判断是否到达目标位置
        if self.arrive_at_cart_pos():
            self.logger(f"  -> 到达抓车位，进入 Act")
            self.transition_step(Step.Act)
        elif self.arrive_at_observe_pos():
            self.logger(f"  -> 到达观察点，进入 Observe")
            self.transition_step(Step.Observe)
    
    def _search_observe(self):
        """观察：检测目标是否存在"""
        if self.observing:
            return
        
        self.logger(f"👁️ [Observe] 开始观察，当前位置: {self.current_pos_symbol}")
        self.observing = True
        self.vision.set_vision_enabled(True)
        
        # 模拟检测延迟
        time.sleep(0.5)
        
        if self.target_found(self.phase):
            # 找到目标
            self.target_cart_pos = self.get_cart_pos_by_symbol(self.current_pos_symbol)
            self.target_pos_symbol = self.target_cart_pos
            self.logger(f"🎯 [Observe] 找到目标车位: {self.target_cart_pos}")
            self.observing = False
            self.vision.set_vision_enabled(False)
            self.logger(f"  -> 进入 Act")
            self.transition_step(Step.Act)
        else:
            # 未找到目标，前往下一个观察点
            next_pos = self.get_next_pos(self.current_pos_symbol)
            self.target_pos_symbol = next_pos
            self.logger(f"🔍 [Observe] 未找到目标，前往下一点: {next_pos}")
            self.observing = False
            self.vision.set_vision_enabled(False)
            self.transition_step(Step.Cruise)
    
    def _act(self):
        """动作：执行抓取/放置"""
        if self.acting:
            return
        
        action = "抓取" if self.phase in [Phase.SearchFull, Phase.SearchEmpty] else "放置"
        self.logger(f"🤖 [Act] 执行 {action} 动作，位置: {self.current_pos_symbol}")
        self.acting = True
        
        # 模拟动作执行
        time.sleep(0.5)
        
        # 模拟动作结果（90%成功率）
        success = random.random() < 0.9
        if success:
            self.logger(f"✅ [Act] {action} 成功！")
            self.acting = False
            self.transition_step(Step.Done)
        else:
            self.logger(f"❌ [Act] {action} 失败！")
            self.acting = False
            self.logger(f"  -> 返回 Cruise 重试")
            self.transition_step(Step.Cruise)
    
    def _run_search_full(self):
        """SearchFull：找满车流程"""
        self.logger(f"\n📋 [SearchFull] 状态机循环: {self.step}")
        
        if self.step == Step.Cruise:
            self._cruise()
        elif self.step == Step.Observe:
            self._search_observe()
        elif self.step == Step.Act:
            self._act()
        elif self.step == Step.Done:
            self.logger("✅ SearchFull 完成！")
            self.transition_phase(Phase.PutFull)
    
    def _run_put_full(self):
        """PutFull：放满车流程"""
        self.logger(f"\n📋 [PutFull] 状态机循环: {self.step}")
        
        if self.step == Step.Cruise:
            self._cruise()
        elif self.step == Step.Observe:
            self._search_observe()
        elif self.step == Step.Act:
            self._act()
        elif self.step == Step.Done:
            self.logger("✅ PutFull 完成！")
            self.transition_phase(Phase.SearchEmpty)
    
    def _run_search_empty(self):
        """SearchEmpty：找空车流程"""
        self.logger(f"\n📋 [SearchEmpty] 状态机循环: {self.step}")
        
        if self.step == Step.Cruise:
            self._cruise()
        elif self.step == Step.Observe:
            self._search_observe()
        elif self.step == Step.Act:
            self._act()
        elif self.step == Step.Done:
            self.logger("✅ SearchEmpty 完成！")
            self.transition_phase(Phase.PutEmpty)
    
    def _run_put_empty(self):
        """PutEmpty：放空车流程"""
        self.logger(f"\n📋 [PutEmpty] 状态机循环: {self.step}")
        
        if self.step == Step.Cruise:
            self._cruise()
        elif self.step == Step.Observe:
            self._search_observe()
        elif self.step == Step.Act:
            self._act()
        elif self.step == Step.Done:
            self.logger("✅ PutEmpty 完成！")
            self.transition_phase(Phase.SearchFull)
    
    def state_machine_loop(self):
        """状态机主循环"""
        if self.phase == Phase.IDLE:
            return
        if self.phase == Phase.ERROR:
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
            self.logger(f"❌ 未知状态: {self.phase}")
    
    def start_task(self):
        """启动任务"""
        self.logger("\n🎬 任务开始！")
        self.current_pos_symbol = self.SYMBOL_LST[self.robot_id][Phase.SearchFull][0]
        self.transition_phase(Phase.SearchFull)
        self.transition_step(Step.Cruise)
    
    def stop_task(self):
        """停止任务"""
        self.logger("\n⏹️ 任务停止")
        self.transition_phase(Phase.IDLE)


def main(args=None):
    """主函数"""
    print("\n" + "=" * 60)
    print("🎯 Dummy Coordinator 完整测试")
    print("模拟所有依赖，从头运行状态机")
    print("=" * 60 + "\n")
    
    # 创建并启动 Coordinator
    coordinator = DummyCoordinator(use_ros2=False)
    coordinator.start_task()
    
    # 运行状态机循环
    cycle_count = 0
    max_cycles = 20  # 最多运行 20 个状态周期
    
    try:
        while cycle_count < max_cycles:
            coordinator.state_machine_loop()
            cycle_count += 1
            print(f"\n--- 周期 {cycle_count}/{max_cycles} ---")
            print(f"当前状态: {coordinator.phase} / {coordinator.step}")
            print(f"当前位置: {coordinator.current_pos_symbol}")
            print(f"目标位置: {coordinator.target_pos_symbol}")
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n⏹️ 收到中断信号")
    finally:
        coordinator.stop_task()
        print("\n" + "=" * 60)
        print("✅ Dummy Coordinator 测试完成")
        print("=" * 60)


if __name__ == "__main__":
    main(sys.argv[1:])
