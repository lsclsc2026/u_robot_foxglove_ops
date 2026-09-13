#!/usr/bin/env python3
"""Navigation Adapter 测试脚本"""

import sys
import time
import rclpy
from rclpy.node import Node

# 导航适配器导入
sys.path.insert(0, '/home/user/hanjiatong/mos_coordinator')
from mos_coordinator.adapters.navigation_adapter import NavigationAdapter


class TestNavigationNode(Node):
    """测试 Navigation Adapter 的节点"""

    def __init__(self):
        super().__init__('test_navigation_adapter')

        # 初始化导航适配器
        self.nav_adapter = NavigationAdapter(self)

        # 设置回调
        self.nav_adapter.on_goal_reached = self._on_goal_reached
        self.nav_adapter.on_goal_failed = self._on_goal_failed
        self.nav_adapter.on_goal_aborted = self._on_goal_aborted

        # 测试序列
        self._test_index = 0
        self._test_goals = [
            (1.0, 2.0, 0.0),
            (3.0, 1.0, 1.57),
            (0.0, 0.0, 0.0),
        ]

        self.get_logger().info("✅ Navigation Adapter 测试节点启动")

        # 启动定时器执行测试
        self._test_timer = self.create_timer(1.0, self._run_test)

    def _run_test(self):
        """执行测试序列"""
        if self._test_index >= len(self._test_goals):
            self.get_logger().info("🎉 所有测试完成！")
            self._test_timer.cancel()
            return

        goal = self._test_goals[self._test_index]
        self.get_logger().info(
            f"[测试 {self._test_index + 1}/{len(self._test_goals)}] "
            f"导航到: ({goal[0]:.1f}, {goal[1]:.1f}), yaw={goal[2]:.2f}")
        self.nav_adapter.navigate_to_pose(goal[0], goal[1], goal[2])
        self._test_index += 1

    def _on_goal_reached(self):
        """导航成功到达回调"""
        self.get_logger().info("✅ [回调] 导航到达目标！")

    def _on_goal_failed(self):
        """导航失败回调"""
        self.get_logger().info("❌ [回调] 导航失败！")

    def _on_goal_aborted(self):
        """导航中止回调"""
        self.get_logger().info("⚠️ [回调] 导航中止！")


def main(args=None):
    rclpy.init(args=args)

    node = TestNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ 测试中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
