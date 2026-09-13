#!/usr/bin/env python3
"""任务协调器模拟测试节点

独立的模拟测试节点，使用递归轮询模式测试找空阶段流程。
与coordinator_node分离，保持生产代码的纯净性。
"""

import rclpy
from rclpy.node import Node
import time

from mos_coordinator.states import TaskState
from mos_coordinator.adapters import NavigationAdapter, VisionAdapter


class SimulatorNode(Node):
    """模拟测试节点"""
    
    def __init__(self):
        super().__init__('simulator_node')
        
        # ===== 初始化适配器 =====
        self.navigation = NavigationAdapter(self)
        self.vision = VisionAdapter(self)
        
        # ===== 状态管理 =====
        self.state = TaskState.SearchEmpty
        self.previous_state = None
        
        # ===== 机器人配置 =====
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
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("模拟测试节点已启动")
        self.get_logger().info("初始状态: SearchEmpty")
        self.get_logger().info("=" * 50)
    
    def transition_to(self, new_state: TaskState, error_msg: str = ""):
        """状态转换"""
        if self.state == new_state:
            return
        
        self.previous_state = self.state
        self.state = new_state
        self.get_logger().info(f"状态转换: {self.previous_state.name} -> {new_state.name}")
    
    def start_simulation(self):
        """启动模拟测试"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("开始模拟测试 - 找空阶段")
        self.get_logger().info("=" * 50)
        self.search_for_empty_cart_simple()
    
    def search_for_empty_cart_simple(self):
        """找空阶段简化模拟

        巡航DE区间：
        1. D↔E循环巡航
        2. 每次到达巡航点检查是否有空车
        3. 检测到空车后，导航到小车位置
        4. 到达小车位置后抓取
        5. 抓取成功后转到放置阶段
        """
        self.get_logger().info('🔍 search_for_empty_cart_simple')
        arrived_point_symbol = self.navigation.arrive_point()
        self.get_logger().info(f'arrived_point = {arrived_point_symbol}')

        # 检查是否正在导航中
        if self.navigation.is_ing():
            self.get_logger().info('⏳ 导航中，等待...')
            time.sleep(10)
            self.transition_to(TaskState.SearchEmpty)
            self.search_for_empty_cart_simple()
            return

        # 检查是否到达小车位置（DCart或ECart）
        if arrived_point_symbol in ['DCart', 'ECart']:
            # 必须先检查是否已经导航到小车位置
            # 只有在 simulation_target 是小车位置时才允许抓取
            if (hasattr(self.navigation, 'simulation_target') and
                self.navigation.simulation_target == arrived_point_symbol):
                # 确认已导航到小车位置，执行抓取
                self.get_logger().info(f'✅ 到达小车位置 {arrived_point_symbol}')
                if self.vision.fetch():
                    # 抓取成功，转到放置阶段
                    self.get_logger().info('✅ 抓取成功')
                    self.get_logger().info('=' * 50)
                    self.get_logger().info('🎉 找空阶段完成！转到放置阶段')
                    self.get_logger().info('=' * 50)
                    self.transition_to(TaskState.SearchPutEmptyPos)
                    # 找空阶段结束，不再继续
                    return
                else:
                    self.get_logger().error("❌ 抓取失败")
                    self.transition_to(TaskState.ERROR)
                    return
            else:
                # 随机到达了小车位置，但并非我们的目标，视为异常点，继续巡航
                self.get_logger().warn(f'⚠️  随机到达 {arrived_point_symbol}，但未导航过去，继续巡航')
                next_point = 'D'  # 重新从D开始
                self.navigation.go_to_point(next_point)
                self.transition_to(TaskState.SearchEmpty)
                self.search_for_empty_cart_simple()
            return

        # 到达巡航点（D或E），检查装载状态
        if arrived_point_symbol in ['D', 'E']:
            self.get_logger().info(f'📍 巡航点 {arrived_point_symbol} - 检查装载状态')

            # 检查是否有空车
            if self.vision.fullness_result() is False:  # 检测到空车
                # 导航到对应的小车位置
                target_cart = arrived_point_symbol + 'Cart'  # D→DCart, E→ECart
                self.get_logger().info(f'🎯 发现空车！导航到 {target_cart}')
                self.navigation.go_to_point(target_cart)
                self.transition_to(TaskState.SearchEmpty)
                self.search_for_empty_cart_simple()
            else:
                # 没有空车，继续下一个巡航点
                self.get_logger().info('➡️  满车或无车，继续巡航')
                next_point = self.get_next_patrol_point(arrived_point_symbol)
                self.get_logger().info(f'📍 下一个巡航点: {next_point}')
                self.navigation.go_to_point(next_point)
                self.transition_to(TaskState.SearchEmpty)
                self.search_for_empty_cart_simple()
            return

        # 其他情况：继续巡航
        self.get_logger().warn(f'⚠️  未知点 {arrived_point_symbol}，从D开始巡航')
        next_point = 'D'  # 默认从D开始
        self.navigation.go_to_point(next_point)
        self.transition_to(TaskState.SearchEmpty)
        self.search_for_empty_cart_simple()
    
    def get_next_patrol_point(self, current_point):
        """获取下一个巡航点（D↔E循环）"""
        patrol_route = self.SYMBOL_LST[self.robot_id]['SearchEmpty']  # ['D', 'E']
        if current_point == patrol_route[0]:
            return patrol_route[1]  # D→E
        else:
            return patrol_route[0]  # E→D

    def on_cart_fullness_received(self, is_full: bool):
        """接收装载检测结果的回调（VisionAdapter需要）

        模拟测试中不需要处理，只是为了兼容VisionAdapter
        """
        pass

    def on_cart_3d_detected(self, cart_pose, class_id: int):
        """接收3D检测结果的回调（VisionAdapter需要）

        模拟测试中不需要处理，只是为了兼容VisionAdapter
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    simulator = SimulatorNode()
    
    # 启动模拟测试
    simulator.start_simulation()
    
    # ROS2 spin (实际上不会到达这里，因为模拟会exit)
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
