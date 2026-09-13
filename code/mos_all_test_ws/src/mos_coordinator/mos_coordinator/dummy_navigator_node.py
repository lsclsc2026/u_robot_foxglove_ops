#!/usr/bin/env python3
"""Dummy Navigator Node - 随机发布导航状态

ROS2 节点，模拟导航器的行为：
1. 订阅 /goal_pose 接收导航目标
2. 随机发布导航状态（到达成功/失败）
3. 模拟真实导航的异步行为

用于测试 Coordinator 的导航状态处理逻辑。
"""

import random
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String


class DummyNavigatorNode(Node):
    """随机发布导航状态的 ROS2 节点"""

    def __init__(self):
        super().__init__('dummy_navigator_node')
        
        # ===== 声明参数 =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('success_rate', 0.9),
                ('min_delay', 2.0),
                ('max_delay', 4.0),
                ('goal_topic', '/goal_pose'),
                ('status_topic', '/navigator/status'),
                ('localization_topic', '/localization'),
            ])
        
        # ===== 获取参数 =====
        self._success_rate = self.get_parameter('success_rate').get_parameter_value().double_value
        self._min_delay = self.get_parameter('min_delay').get_parameter_value().double_value
        self._max_delay = self.get_parameter('max_delay').get_parameter_value().double_value
        self._goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self._status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self._localization_topic = self.get_parameter('localization_topic').get_parameter_value().string_value
        
        # ===== 订阅导航目标 =====
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self._goal_topic,
            self._goal_callback,
            10)
        
        # ===== 发布导航状态 =====
        self.status_pub = self.create_publisher(
            String,
            self._status_topic,
            10)
        
        # ===== 发布当前位姿 =====
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self._localization_topic,
            10)
        
        # ===== 状态发布定时器 =====
        self.status_timer = self.create_timer(1.0, self._publish_status)
        
        # ===== 导航状态 =====
        self._current_goal = None
        self._current_pose = PoseStamped()
        self._current_pose.header.frame_id = 'map'
        self._current_pose.pose.orientation.w = 1.0  # 默认朝向
        self._navigation_active = False
        self._navigation_result = None  # 'success', 'failed', None
        
        self.get_logger().info("✅ Dummy Navigator Node 启动")
        self.get_logger().info(f"   成功率: {self._success_rate*100:.0f}%")
        self.get_logger().info(f"   延迟范围: {self._min_delay}-{self._max_delay}秒")
        self.get_logger().info(f"   目标话题: {self._goal_topic}")
        self.get_logger().info(f"   状态话题: {self._status_topic}")
        self.get_logger().info(f"   定位话题: {self._localization_topic}")
        
        # 启动一个线程处理导航结果
        import threading
        self._nav_thread = threading.Thread(target=self._navigation_worker, daemon=True)
        self._nav_thread.start()

    def _goal_callback(self, msg: PoseStamped):
        """接收导航目标"""
        self.get_logger().info(
            f"📍 收到导航目标: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        
        self._current_goal = msg
        self._navigation_active = True
        self._navigation_result = None
        
        # 模拟导航处理（在工作线程中执行）
        self._schedule_navigation_result()

    def _schedule_navigation_result(self):
        """安排导航结果发布（模拟异步导航）"""
        delay = random.uniform(self._min_delay, self._max_delay)
        self.get_logger().info(f"⏳ 模拟导航中，预计 {delay:.1f} 秒后到达")
        
        # 使用 threading.Timer 延迟发布结果（ROS2 Humble 不支持 oneshot 参数）
        import threading
        timer = threading.Timer(delay, self._publish_navigation_result)
        timer.start()

    def _publish_navigation_result(self):
        """发布导航结果（成功或失败）"""
        if not self._navigation_active or self._current_goal is None:
            return
        
        # 随机决定成功或失败
        if random.random() < self._success_rate:
            self._navigation_result = 'success'
            # 更新当前位姿为目标位姿
            self._current_pose = self._current_goal
            self.get_logger().info("✅ 导航成功到达目标！")
        else:
            self._navigation_result = 'failed'
            self.get_logger().info("❌ 导航失败！")
        
        self._navigation_active = False
        
        # 发布状态
        self._publish_status()

    def _publish_status(self):
        """发布导航状态"""
        status_msg = String()
        
        if self._navigation_active:
            status_msg.data = 'navigating'
        elif self._navigation_result == 'success':
            status_msg.data = 'success'
        elif self._navigation_result == 'failed':
            status_msg.data = 'failed'
        else:
            status_msg.data = 'idle'
        
        self.status_pub.publish(status_msg)
        
        # 发布当前位姿（模拟定位信息）
        self._current_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self._current_pose)

    def _navigation_worker(self):
        """导航工作线程（保持节点活跃）"""
        while rclpy.ok():
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    
    node = DummyNavigatorNode()
    
    # 使用多线程执行器，处理异步导航
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ 收到中断信号")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
