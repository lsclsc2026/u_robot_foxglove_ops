#!/usr/bin/env python3
"""导航子系统适配器（基于 MOS Nav2）

纯功能适配器，不包含业务逻辑
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# === 尝试导入 Nav2 相关模块 ===
try:
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    from tf_transformations import quaternion_from_euler
    NAV2_AVAILABLE = True
except ImportError:
    NAV2_AVAILABLE = False
    ActionClient = None
    NavigateToPose = None
    GoalStatus = None
    quaternion_from_euler = lambda r, p, y: (0.0, 0.0, float(math.sin(y/2)), float(math.cos(y/2)))
#NAV2_AVAILABLE = False

# === 底盘控制服务（独立于导航，可选） ===
try:
    from mos_chassis_controller.srv import MoveChassis
except ImportError:
    MoveChassis = None


class NavigationAdapter:
    """导航适配器：基于 Nav2 的导航接口

    纯功能接口，不包含业务逻辑：
    1. 发送导航目标
    2. 订阅当前位姿（从 /localization）
    3. 到达检测（从 Nav2 Action 状态）
    4. 底盘对准（独立功能）

    如果 Nav2 不可用，会自动使用模拟实现
    """

    def __init__(self, node: Node):
        self.node = node
        self._nav2_available = NAV2_AVAILABLE

        # ===== 订阅当前位姿 (Odometry 类型) =====
        self.localization_sub = node.create_subscription(
            Odometry,
            '/localization',
            self._localization_callback,
            10)

        # ===== Nav2 Action Client =====
        if NAV2_AVAILABLE:
            self._nav_to_pose_client = ActionClient(
                node,
                NavigateToPose,
                'navigate_to_pose')
        else:
            self._nav_to_pose_client = None
            node.get_logger().warn('⚠️ Nav2 不可用，使用模拟导航')

        # ===== 发布导航目标 (兼容性保留) =====
        self.goal_pub = node.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)

        # ===== 底盘控制服务（独立于导航，用于对准抓取） =====
        if MoveChassis is not None:
            self.chassis_client = node.create_client(
                MoveChassis,
                '/mos_chassis_controller/run_chassis'
            )
        else:
            self.chassis_client = None
            node.get_logger().warn('MoveChassis 服务不可用，底盘对准功能将被禁用')

        # ===== 状态变量 =====
        self.current_pose = None  # 当前位姿（从 /localization 更新）
        self.current_goal_handle = None  # 当前导航目标句柄
        self.current_goal_pose = None  # 当前目标位姿

        # ===== 回调函数 =====
        self.on_goal_reached = None  # 到达目标回调
        self.on_goal_failed = None   # 导航失败回调

        # ===== 模拟导航状态（用于 Nav2 不可用的情况）=====
        self._sim_nav_active = False
        self._sim_nav_goal = None

        self.node.get_logger().info('NavigationAdapter initialized')
        self.node.get_logger().info(f'  Nav2可用: {self._nav2_available}')
        self.node.get_logger().info('  订阅: /localization (Odometry)')
        self.node.get_logger().info('  发布: /goal_pose (PoseStamped)')

    def _localization_callback(self, msg: Odometry):
        """接收定位数据"""
        # 转换 Odometry 为 PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.current_pose = pose_stamped

    def _build_goal_pose(
        self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map'
    ) -> PoseStamped:
        """构造 NavigateToPose 目标位姿。"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        return goal_pose

    def _simulate_navigation(self):
        """模拟导航完成（当 Nav2 不可用时）"""
        self._sim_nav_active = False
        self.on_goal_reached = True
        self.node.get_logger().info('✅ [模拟] 导航到达目标点')

    def navigate_to_pose(self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map'):
        """导航到指定位姿（非阻塞：发送目标后立即返回）

        Args:
            x: X坐标（米）
            y: Y坐标（米）
            yaw: 偏航角（弧度）
            frame_id: 坐标系

        Returns:
            True表示目标发送成功，False表示失败
        """
        goal_pose = self._build_goal_pose(x, y, yaw, frame_id)
        self.current_goal_pose = goal_pose
        self.goal_pub.publish(goal_pose)

        # ===== Nav2 不可用：使用模拟导航 =====
        if not self._nav2_available:
            self.node.get_logger().info(f'📍 [模拟] 导航到: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°')
            self._sim_nav_goal = (x, y, yaw)
            # 立即模拟到达
            self._simulate_navigation()
            return True

        # ===== Nav2 可用：使用真实导航 =====
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn('NavigateToPose action服务器不可用，仅发布到/goal_pose')
            return True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

        self.node.get_logger().info(f'📍 导航到: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°')
        return True

    def navigate_to_pose_sync(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str = 'map',
        timeout_sec: float = 120.0,
    ) -> bool:
        """导航到指定位姿，阻塞直到到达（或失败/超时）才返回。

        使用 Nav2 Action 的 async API，并通过 spin_until_future_complete 等待结果。
        若从本节点的单线程 executor 回调中调用，请改用 MultiThreadedExecutor，
        否则可能因无法处理 action 回调而死锁。

        如果 Nav2 不可用，会使用模拟导航（立即返回成功）

        Args:
            x: X坐标（米）
            y: Y坐标（米）
            yaw: 偏航角（弧度）
            frame_id: 坐标系
            timeout_sec: 等待接受目标与导航结果的总超时（秒）

        Returns:
            True 表示导航成功到达；False 表示拒绝、失败或超时
        """
        self.node.get_logger().info(
            f'📍 导航(阻塞等待): ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°, {frame_id}')
        goal_pose = self._build_goal_pose(x, y, yaw, frame_id)
        self.current_goal_pose = goal_pose
        self.goal_pub.publish(goal_pose)

        # ===== Nav2 不可用：使用模拟导航 =====
        if not self._nav2_available:
            self.node.get_logger().info('✅ [模拟] 导航成功（Nav2不可用）')
            return True

        # ===== Nav2 可用：使用真实导航 =====
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error('NavigateToPose action服务器不可用')
            if self.on_goal_failed:
                self.on_goal_failed()
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self.node, send_goal_future, timeout_sec=timeout_sec
        )

        if not send_goal_future.done():
            self.node.get_logger().error('❌ 等待导航目标接受超时')
            if self.on_goal_failed:
                self.on_goal_failed()
            return False

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error('❌ 导航目标被拒绝')
            if self.on_goal_failed:
                self.on_goal_failed()
            return False

        self.node.get_logger().info('✅ 导航目标已接受，等待到达...')
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=timeout_sec
        )

        if not result_future.done():
            self.node.get_logger().error('❌ 等待导航结果超时，取消导航')
            goal_handle.cancel_goal_async()
            self.current_goal_handle = None
            if self.on_goal_failed:
                self.on_goal_failed()
            return False

        status = result_future.result().status
        self.current_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('✅ Nav2 Action: 导航成功（阻塞等待结束）')
            return True

        self.node.get_logger().warn(f'⚠️ 导航失败: status={status}')
        if self.on_goal_failed:
            self.on_goal_failed()
        return False

    def _goal_response_callback(self, future):
        """导航目标响应回调"""
        if not self._nav2_available:
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('❌ 导航目标被拒绝')
            if self.on_goal_failed:
                self.on_goal_failed()
            return

        self.node.get_logger().info('✅ 导航目标已接受')
        self.current_goal_handle = goal_handle

        # 等待结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """导航结果回调"""
        if not self._nav2_available:
            return

        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('✅ Nav2 Action: 导航成功')
            # 触发到达回调（不依赖外部）
            self.on_goal_reached = True
        else:
            self.on_goal_failed = True
            self.node.get_logger().warn(f'⚠️ 导航失败: status={status}')

        self.current_goal_handle = None

    def cancel_navigation(self):
        """取消当前导航任务并停止底盘（增强诊断版）

        三重保险机制：
        1. 通过 goal_handle 取消
        2. 通过 CancelGoal 服务取消所有任务
        3. 发送停止命令到底盘（30次）

        Returns:
            bool: True表示成功取消
        """
        self.node.get_logger().warn('='*70)
        self.node.get_logger().warn('🛑 开始执行导航取消 🛑')
        self.node.get_logger().warn('='*70)

        cancelled = False

        # 方法1：goal_handle
        self.node.get_logger().info('[1/3] 尝试通过 goal_handle 取消...')
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
                cancelled = True
                self.node.get_logger().info('  ✅ goal_handle 取消成功')
            except Exception as e:
                self.node.get_logger().error(f'  ❌ goal_handle 取消失败: {e}')
        else:
            self.node.get_logger().warn('  ⚠️ goal_handle 为空，跳过')

        # 方法2：CancelGoal 服务
        self.node.get_logger().info('[2/3] 尝试通过 CancelGoal 服务取消...')
        from action_msgs.srv import CancelGoal
        from action_msgs.msg import GoalInfo
        from unique_identifier_msgs.msg import UUID

        cancel_client = self.node.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal'
        )

        if cancel_client.wait_for_service(timeout_sec=1.0):
            try:
                cancel_request = CancelGoal.Request()
                cancel_request.goal_info = GoalInfo()
                cancel_request.goal_info.goal_id = UUID()

                cancel_future = cancel_client.call_async(cancel_request)
                rclpy.spin_until_future_complete(
                    self.node,
                    cancel_future,
                    timeout_sec=3.0
                )

                if cancel_future.done():
                    response = cancel_future.result()
                    goal_count = len(response.goals_canceling) if response else 0
                    self.node.get_logger().info(f'  CancelGoal 响应：{goal_count} 个目标被取消')
                    if goal_count > 0:
                        cancelled = True
                        self.node.get_logger().info(f'  ✅ 成功取消 {goal_count} 个导航任务')
                    else:
                        self.node.get_logger().warn('  ⚠️ 没有找到活动的导航任务')
                else:
                    self.node.get_logger().warn('  ⚠️ CancelGoal 请求超时')
            except Exception as e:
                self.node.get_logger().error(f'  ❌ CancelGoal 调用异常: {e}')
        else:
            self.node.get_logger().warn('  ⚠️ CancelGoal 服务不可用')

        # 方法3：停止底盘
        self.node.get_logger().info('[3/3] 发送停止命令到底盘...')
        from geometry_msgs.msg import Twist
        import time

        if not hasattr(self, '_cmd_vel_pub'):
            self._cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        stop_msg = Twist()
        send_count = 0
        for i in range(30):  # 发送30次，提高可靠性
            self._cmd_vel_pub.publish(stop_msg)
            if i % 10 == 0:
                self.node.get_logger().info(f'  发送停止命令 {i+1}/30')
            time.sleep(0.02)  # 每20ms一次
            rclpy.spin_once(self.node, timeout_sec=0.01)
            send_count += 1

        self.node.get_logger().info(f'  ✅ 已发送 {send_count} 次停止命令')

        # 清理状态
        self.current_goal_handle = None

        self.node.get_logger().warn('='*70)
        self.node.get_logger().warn(f'🛑 导航取消完成，结果={cancelled} 🛑')
        self.node.get_logger().warn('='*70)

        return cancelled or True  # 即使没有导航任务，发送停止命令也算成功

    def get_current_pose(self) -> PoseStamped:
        """获取当前位姿"""
        return self.current_pose

    # ===== 底盘对准功能（独立） =====

    def align_chassis(self,target_x_m: float = None,return_to_origin: bool = False,direction: str = 'left',):
        """调用底盘对准服务

        Args:
            direction: 对准方向 'left' 或 'right'
        """
        if self.chassis_client is None:
            self.node.get_logger().error('MoveChassis 服务不可用')
            return False

        # # 🎯 测试模式：直接模拟对准成功
        # import time
        # time.sleep(0.5)  # 模拟底盘移动延时
        # self.node.get_logger().info(f'✅ [测试模式] 底盘对准完成: {direction}')
        # self._alignment_service_callback(None)  # 直接调用回调模拟成功
        # return True

        # 原有服务调用逻辑（测试时不执行）
        request = MoveChassis.Request()
        request.target_x_m = float(target_x_m) if target_x_m is not None else 0.0
        request.return_to_origin = return_to_origin

        future = self.chassis_client.call_async(request)
        future.add_done_callback(self._alignment_service_callback)

        self.node.get_logger().info(f'🎯 开始底盘对准: {direction}')
        return True

    def _alignment_service_callback(self, future):
        """底盘对准服务响应回调"""
        # 🎯 测试模式：future=None 时直接返回成功
        if future is None:
            if hasattr(self.node, 'on_alignment_complete'):
                self.node.on_alignment_complete(True)
            return
        
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f'✅ 底盘对准完成: {response.message}')
                # 通知 coordinator
                if hasattr(self.node, 'on_alignment_complete'):
                    self.node.on_alignment_complete(True)
            else:
                self.node.get_logger().warn(f'❌ 底盘对准失败: {response.message}')
                if hasattr(self.node, 'on_alignment_complete'):
                    self.node.on_alignment_complete(False)
        except Exception as e:
            self.node.get_logger().error(f'❌ 底盘对准服务调用异常: {e}')
            if hasattr(self.node, 'on_alignment_complete'):
                self.node.on_alignment_complete(False)
    
    
