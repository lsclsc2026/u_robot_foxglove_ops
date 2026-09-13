#!/usr/bin/env python3
"""机械臂子系统适配器（最简骨架）"""

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

# === Dummy 测试配置 ===
# from mos_coordinator.srv import RunTask

# === 真实功能包配置（可选导入）===
try:
    from mos_arm_controller.srv import RunTask
except ImportError:
    RunTask = None


class ArmAdapter:
    """机械臂适配器 : 处理机械臂抓取目标点的发布。"""

    def __init__(self, node: Node):
        self.node = node

        # ===== Service Clients =====
        # === Dummy 测试配置（注释掉） ===
        # from mos_coordinator.srv import RunTask  # 导入 coordinator 包的服务定义
        # self.arm_client = node.create_client(
        #     RunTask,
        #     '/dummy_arm/run_task'
        # )

        # === 真实功能包配置（可选）===
        # 注意：真实功能包使用自己的服务定义，需要修改导入语句
        # 在文件顶部添加: from mos_arm_controller.srv import RunTask
        if RunTask is not None:
            self.arm_client = node.create_client(
                RunTask,  # 使用 mos_arm_controller 包的 RunTask
                '/mos_arm_controller/run_task'  # 真实机械臂控制器服务
            )
        else:
            self.arm_client = None
            node.get_logger().warn('RunTask 服务不可用，机械臂功能将被禁用')

        # 暂时保留 release 发布器（后续改为 service）
        self.release_pub = node.create_publisher(
            Bool,
            '/arm/release_cart',
            10)

        # ===== 后台线程支持 =====
        self._put_thread = None
        self._put_result = None
        self._put_timeout_sec = 60.0
        self._put_event = threading.Event()
        self._put_success = False

        self.node.get_logger().info('ArmAdapter initialized')

    def grasp_at_point(self, left_point: PointStamped, right_point: PointStamped):
        """发送抓取目标点给机械臂（调用 RunTask 服务）。

        Args:
            left_point: 左手把手位置
            right_point: 右手把手位置
        """
        if self.arm_client is None:
            self.node.get_logger().warn('机械臂服务不可用')
            return

        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('机械臂服务未就绪')
            return
        
        request = RunTask.Request()
        request.task_name = 'grasp'
        request.target_frame = left_point.header.frame_id
        
        # ========== Dummy 测试用硬编码坐标（注释掉）==========
        # test_left = PointStamped()
        # test_left.header.frame_id = 'body_link'
        # test_left.point.x = 0.6
        # test_left.point.y = 0.3
        # test_left.point.z = -0.2
        # 
        # test_right = PointStamped()
        # test_right.header.frame_id = 'body_link'
        # test_right.point.x = 0.6
        # test_right.point.y = -0.3
        # test_right.point.z = -0.2
        # 
        # request.left_point = test_left   
        # request.right_point = test_right 
        # self.node.get_logger().warn('⚠️ 使用测试硬编码坐标（非视觉数据）')
        # ========== 测试用硬编码坐标结束 ==========
        
        # === 真实功能包配置：使用视觉检测的把手位置 ===
        request.left_point = left_point
        request.right_point = right_point
        
        self.node.get_logger().info(
            f'发送抓取请求 -> /arm/run_task: '
            f'左手({request.left_point.point.x:.3f}, {request.left_point.point.y:.3f}, {request.left_point.point.z:.3f}), '
            f'右手({request.right_point.point.x:.3f}, {request.right_point.point.y:.3f}, {request.right_point.point.z:.3f})'
        )
        
        # 异步调用
        future = self.arm_client.call_async(request)
        future.add_done_callback(self._arm_service_callback)
    
    def release_cart(self):
        """请求机械臂释放小车。"""
        msg = Bool()
        msg.data = True
        self.release_pub.publish(msg)
        self.node.get_logger().info('已发布释放小车指令 -> /arm/release_cart')
    
    def _arm_service_callback(self, future):
        """机械臂任务服务响应回调"""
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f'✅ 机械臂任务完成: {response.message}')
            else:
                self.node.get_logger().warn(f'❌ 机械臂任务失败: {response.message}')
            # 触发 coordinator 状态转换
            self.node.on_grasp_result(response.success)
        except Exception as e:
            self.node.get_logger().error(f'机械臂服务调用异常: {e}')
            self.node.on_grasp_result(False)

    def put_async(self, timeout_sec: float = 60.0):
        """非阻塞版本：在后台线程执行 place 任务。

        立即返回，放置流程在后台线程运行。
        使用 is_put_running() 检查状态，get_put_result() 获取结果。
        """
        if self._put_thread is not None and self._put_thread.is_alive():
            self.node.get_logger().warn('Put 已在运行中，忽略重复调用')
            return

        self._put_result = None
        self._put_timeout_sec = timeout_sec
        self._put_thread = threading.Thread(
            target=self._put_with_result,
            daemon=True,
            name='PutAdapter'
        )
        self._put_thread.start()
        self.node.get_logger().info('🚀 Put 后台线程已启动')

    def _put_with_result(self):
        """内部：执行 put 并保存结果"""
        try:
            self._put_result = self._put_sync(self._put_timeout_sec)
        except Exception as e:
            self.node.get_logger().error(f'Put 后台线程异常: {e}')
            self._put_result = False

    def _put_sync(self, timeout_sec: float = 60.0) -> bool:
        """同步请求机械臂 place 任务（后台线程内调用，用 Event 代替 spin）"""
        if self.arm_client is None:
            self.node.get_logger().warn('机械臂服务不可用')
            return False
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('机械臂服务未就绪')
            return False

        request = RunTask.Request()
        request.task_name = 'place'

        self._put_event.clear()
        self._put_success = False
        future = self.arm_client.call_async(request)
        future.add_done_callback(self._put_done_callback)

        self.node.get_logger().info(f'已请求机械臂 place 任务 (超时 {timeout_sec}s)')

        # 阻塞等待 event (在后台线程里, 不影响主线程执行器)
        if not self._put_event.wait(timeout=timeout_sec):
            self.node.get_logger().error(f'❌ 放置超时 (>{timeout_sec}s)')
            return False

        return self._put_success

    def _put_done_callback(self, future):
        """place 服务的响应回调（由执行器在主线程调用）"""
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f'✅ 放置成功: {response.message}')
                self._put_success = True
            else:
                self.node.get_logger().warn(f'❌ 放置失败: {response.message}')
                self._put_success = False
        except Exception as e:
            self.node.get_logger().error(f'机械臂 place 服务调用异常: {e}')
            self._put_success = False
        finally:
            self._put_event.set()

    def is_put_running(self) -> bool:
        """检查放置流程是否仍在运行"""
        return self._put_thread is not None and self._put_thread.is_alive()

    def get_put_result(self):
        """获取放置结果：True/False/None(尚未完成)"""
        if self.is_put_running():
            return None
        return self._put_result


def main(args=None):
    """独立测试入口: 测试 arm_adapter.put()。

    使用方法:
        1. 启动机械臂服务端: ros2 launch mos_arm_controller hardware_controller.launch.py
        2. 运行本节点: ros2 run mos_coordinator test_arm_adapter
        (需在 setup.py 注册 entry_point)
    """
    rclpy.init(args=args)
    node = Node('test_arm_adapter')
    arm = ArmAdapter(node)

    # 等 2 秒让 service client 初始化完成
    node.get_logger().info('等待 2 秒让 service client 初始化...')
    time.sleep(2)

    # 测试 place
    node.get_logger().info('测试 put...')
    arm.put_async(timeout_sec=60.0)

    # 轮询等待结果
    while arm.is_put_running():
        node.get_logger().info('等待 put 完成...')
        time.sleep(1.0)

    ok = arm.get_put_result()
    node.get_logger().info(f'put 结果: {ok}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
