#!/usr/bin/env python3
"""视觉子系统适配器（最简骨架）"""

import json
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import PointStamped
from std_srvs.srv import SetBool, Trigger  

try:
    from mos_3d_object.msg import DetectedObject3DArray
except ImportError:
    DetectedObject3DArray = None

try:
    from mos_grasp_selector.msg import GraspTaskArray
except ImportError:
    GraspTaskArray = None  


class VisionAdapter:
    """视觉适配器：处理距离检测和把手位置检测。"""

    def __init__(self, node: Node):
        self.node = node

        # ===== 订阅视觉输出 =====
        # 订阅 3D 物体检测结果（持续运行）
        if DetectedObject3DArray is not None:
            # === Dummy 测试配置（注释掉） ===
            # self.detections_3d_sub = node.create_subscription(
            #     DetectedObject3DArray,
            #     '/vision/detections_3d',
            #     self._on_detections_3d,
            #     10)
            
            # === 真实功能包配置 ===
            self.detections_3d_sub = node.create_subscription(
                DetectedObject3DArray,
                '/mos_3d_object/detections_3d',
                self._on_detections_3d,
                10)
        else:
            node.get_logger().warn('DetectedObject3DArray 消息类型不可用，跳过 3D 检测订阅')
        
        # 订阅装载检测结果（JSON 格式）
        self.fullness_result_sub = node.create_subscription(
            String,
            '/cart_fullness/route_signal',
            self._on_fullness_result,
            10)

        # 订阅空车/无车状态（JSON 格式）
        self.cart_status_sub = node.create_subscription(
            String,
            '/cart_fullness/status',
            self._on_cart_status,
            10)

        # 订阅抓取任务（GraspTaskArray）
        if GraspTaskArray is not None:
            self.grasp_tasks_sub = node.create_subscription(
                GraspTaskArray,
                '/vision/grasp_tasks',
                self._grasp_tasks_callback,
                10)
        else:
            node.get_logger().warn('GraspTaskArray 消息类型不可用，跳过抓取任务订阅')

        # ===== 发布视觉请求 =====
        # 发布装载检测航点（激活 cart_fullness）
        self.fullness_waypoint_pub = node.create_publisher(
            String,
            '/cart_fullness/set_waypoint',
            10)
        
        # 请求检测把手
        self.detect_handles_pub = node.create_publisher(
            Bool,
            '/vision/detect_handles',
            10)
        
        # ===== 服务客户端（控制视觉系统开关） =====
        # 控制 3D 检测节点
        self.vision_enable_client = node.create_client(
            SetBool,
            '/mos_3d_object/set_enabled' 
        )
        
        # 控制抓取选择器
        self.selector_enable_client = node.create_client(
            SetBool,
            '/mos_grasp_selector/set_enabled'
        )
        
        # 重置抓取选择器
        self.selector_reset_client = node.create_client(
            Trigger,
            '/mos_grasp_selector/reset'
        )

        self.node.get_logger().info('VisionAdapter initialized')
        # 最近一次有效结果：full/not_full/empty/no_cart。
        # 尚未收到有效结果时保持 None，避免误判。
        self._cart_result = None
        self.fullness_result_record = False  # 用于记录装载检测结果
        self.decided_cart_status_and_rslt_flag = None

    def observe(self):
        self.request_fullness_check("left")
        import time
        time.sleep(0.5)
        return True

    def cart_result(self):
        import random
        # 70%概率是有车位(40%概率满车，20%概率非满车，10%概率空车，30%概率是空车位（临时提高概率用于测试）
        result = random.randint(0, 9) # True = 有车位, False 空车位
        if result < 4:
            self.node.get_logger().info('🎯 检测到空车位！')
            return "full"
        elif result < 6:
            self.node.get_logger().info('🎯 检测到非满车！')
            return "not_full"
        elif result < 7:
            self.node.get_logger().info('🎯 检测到空车！')
            return "empty"
        else:
            self.node.get_logger().info('🎯 检测到无车位！')
            return "no_cart"

    def reset_cart_result(self):
        """清除当前巡检点的结果，等待下一条视觉消息。"""
        self._cart_result = None
        self.fullness_result_record = False

    def fetch(self):
        """模拟方法：执行抓取

        模拟抓取过程，随机等待后返回成功
        """
        self.node.get_logger().info('fake fetch')
        import time
        import random
        # 模拟抓取等待过程
        for _ in range(random.randint(5, 15)):
            self.node.get_logger().info('waiting fetch_done topic')
            time.sleep(0.5)
        return True

    def _on_detections_3d(self, msg):
        """接收 3D 检测结果（持续）"""
        # 只处理小车类别（class_id 0-5）
        carts = [d for d in msg.detections if d.class_id in {0, 1, 2, 3, 4, 5}]
        
        if not carts:
            return
        
        # 取第一个检测到的小车
        cart = carts[0]
        
        # 提取小车位置（已经在 base_link/body_link 坐标系下）
        cart_pose = cart.position_target.point  # x, y, z
        
        # self.node.get_logger().info(
        #     f'检测到小车 class_id={cart.class_id}, '
        #     f'位置=({cart_pose.x:.2f}, {cart_pose.y:.2f}, {cart_pose.z:.2f})'
        # )
        
        # 通知 coordinator
        self.node.on_cart_3d_detected(cart_pose, cart.class_id)
    
    def _on_fullness_result(self, msg: String):
        """接收装载检测结果（JSON 格式）"""
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.node.get_logger().error(f'解析装载检测结果失败: {e}')
            return
        
        decision = data.get('decision')  # "full" 或 "not_full"
        if decision not in {'full', 'not_full'}:
            self.node.get_logger().warn(
                f'忽略 route_signal 未知 decision: {decision}'
            )
            return

        self._cart_result = decision
        is_full = (decision == 'full')
        self.fullness_result_record = is_full
        
        occupancy = data.get('occupancy_ratio', 0.0)
        item_count = data.get('used_item_count', 0)
        
        self.node.get_logger().info(
            f'装载检测结果: {decision}, '
            f'占据率={occupancy:.2f}, 物品数={item_count}'
        )
        self.decided_cart_status_and_rslt_flag = (decision, is_full)
        
        # SearchFullHandler 会在 Observe 子状态中读取 cart_result()。
        # 这里不再调用旧版 coordinator.on_cart_fullness_received()，
        # 避免新旧两套状态机同时处理同一条视觉结果。

    def _on_cart_status(self, msg: String):
        """接收空车/无车状态（JSON 格式）。"""
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.node.get_logger().error(f'解析小车状态失败: {e}')
            return

        decision_map = {
            'empty_cart': 'empty',
            'no_cart': 'no_cart',
        }
        decision = data.get('decision')
        result = decision_map.get(decision)
        if result is None:
            self.node.get_logger().warn(
                f'忽略 status 未知 decision: {decision}'
            )
            return

        self._cart_result = result
        self.node.get_logger().info(
            f'小车状态结果: {decision} -> {result}'
        )
        self.decided_cart_status_and_rslt_flag = (self._cart_result, None)

    def _cart_distance_callback(self, msg: Float32):
        """接收小车距离 -> 通知 coordinator"""
        self.node.get_logger().info(f'检测到小车，距离: {msg.data:.2f}m')
        self.node.on_cart_distance_received(msg.data)

    def _grasp_tasks_callback(self, msg):
        """接收抓取任务 (GraspTaskArray) -> 通知 coordinator

        消息结构：
        tasks:
        - ready: true
          task_name: loaded_treatment_cart_lift
          grasp_mode: dual_arm_sync
          points:
          - role: left_handle
            point: PointStamped
          - role: right_handle
            point: PointStamped
        """
        if not msg.tasks or len(msg.tasks) == 0:
            self.node.get_logger().warn('收到空任务列表')
            return
        
        task = msg.tasks[0]  # 取第一个任务
        if not task.ready:
            self.node.get_logger().warn('任务未就绪')
            return
        
        if task.grasp_mode != 'dual_arm_sync':
            self.node.get_logger().warn(f'不支持的抓取模式: {task.grasp_mode}')
            return
        
        # 提取左右手把手
        left_handle = None
        right_handle = None
        for point_data in task.points:
            if point_data.role == 'left_handle':
                left_handle = point_data.point
            elif point_data.role == 'right_handle': 
                right_handle = point_data.point
        
        if left_handle is None or right_handle is None:
            self.node.get_logger().error('缺少左手或右手把手坐标')
            return
        
        self.node.get_logger().info(
            f'收到抓取任务: {task.task_name}, 模式: {task.grasp_mode}'
        )
        
        # 通知 coordinator
        if hasattr(self.node, 'on_handle_positions_received'):
            self.node.on_handle_positions_received(
                left_handle, right_handle, task.task_name
            )
    
    
    def request_handle_detection(self):
        """请求视觉系统检测把手位置（测试模式：立即返回硬编码位置）"""
        msg = Bool()
        msg.data = True
        self.detect_handles_pub.publish(msg)
        self.node.get_logger().info('已发布把手检测请求 -> /vision/detect_handles')
        
        # # 🎯 测试模式：立即返回硬编码把手位置（照抄dummy数据）
        # import time
        # time.sleep(0.1)  # 短暂延时模拟检测时间
        
        # # 构造硬编码把手位置（与dummy一致）
        # from geometry_msgs.msg import PointStamped
        # from builtin_interfaces.msg import Time
        
        # left_handle = PointStamped()
        # left_handle.header.frame_id = 'body_link'
        # left_handle.header.stamp = self.node.get_clock().now().to_msg()
        # left_handle.point.x = 1.5  # 前方1.5米
        # left_handle.point.y = 0.3  # 左侧0.3米
        # left_handle.point.z = 0.8  # 高0.8米
        
        # right_handle = PointStamped()
        # right_handle.header.frame_id = 'body_link'
        # right_handle.header.stamp = self.node.get_clock().now().to_msg()
        # right_handle.point.x = 1.5   # 前方1.5米
        # right_handle.point.y = -0.3  # 右侧0.3米
        # right_handle.point.z = 0.8   # 高0.8米
        
        # self.node.get_logger().info(
        #     f'🎯 [测试模式] 返回硬编码把手位置: '
        #     f'左({left_handle.point.x:.2f}, {left_handle.point.y:.2f}, {left_handle.point.z:.2f}), '
        #     f'右({right_handle.point.x:.2f}, {right_handle.point.y:.2f}, {right_handle.point.z:.2f})'
        # )
        
        # # 通知coordinator（立即触发）
        # if hasattr(self.node, 'on_handle_positions_received'):
        #     self.node.on_handle_positions_received(
        #         left_handle, right_handle, 'test_grasp'
        #     )
    
    def request_fullness_check(self, side: str):
        """请求装载检测
        
        Args:
            side: 小车所在侧（"left" 或 "right"）
        """
        # 每次发起新的检测请求前丢弃上一巡检点的结果，避免旧结果
        # 被下一个巡检点误用。
        self.reset_cart_result()

        # 根据小车所在侧选择相机
        if side == "left":
            waypoint = "left_start"
        else:
            waypoint = "right_start"
        
        msg = String()
        msg.data = waypoint
        
        self.fullness_waypoint_pub.publish(msg)
        self.node.get_logger().info(f'请求装载检测: waypoint={waypoint} (side={side})')
    
    # ===== 视觉系统控制接口 =====
    
    def set_vision_enabled(self, enabled: bool):
        """控制 3D 检测节点开关
        
        Args:
            enabled: True 启用，False 禁用
        """
        if not self.vision_enable_client.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().warn('3D 检测服务未就绪，跳过开关控制')
            return
        
        request = SetBool.Request()
        request.data = enabled
        
        action = "启用" if enabled else "禁用"
        self.node.get_logger().info(f'{action} 3D 检测节点')
        
        # 异步调用（不等待响应）
        self.vision_enable_client.call_async(request)
    
    def set_selector_enabled(self, enabled: bool):
        """控制抓取选择器开关
        
        Args:
            enabled: True 启用，False 禁用
        """
        if not self.selector_enable_client.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().warn('抓取选择器服务未就绪，跳过开关控制')
            return
        
        request = SetBool.Request()
        request.data = enabled
        
        action = "启用" if enabled else "禁用"
        self.node.get_logger().info(f'{action}抓取选择器')
        
        # 异步调用（不等待响应）
        self.selector_enable_client.call_async(request)
    
    def reset_selector(self):
        """重置抓取选择器（清除历史数据）"""
        if not self.selector_reset_client.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().warn('抓取选择器重置服务未就绪，跳过重置')
            return
        
        request = Trigger.Request()
        self.node.get_logger().info('重置抓取选择器')
        
        # 异步调用（不等待响应）
        self.selector_reset_client.call_async(request)
