#!/usr/bin/env python3
"""仅测试 VisionAdapter 的最小 ROS2 节点。

该节点不创建导航、机械臂或 FetchAdapter，只订阅/触发满载视觉接口，
并把 VisionAdapter 的 cart_result() 变化打印到日志中。
"""

import argparse
import importlib.util
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def _load_vision_adapter():
    """绕过 adapters.__init__ 的 ArmAdapter 导入，单独加载 VisionAdapter。"""
    import mos_coordinator

    module_path = (
        Path(mos_coordinator.__file__).resolve().parent
        / 'adapters'
        / 'vision_adapter.py'
    )
    spec = importlib.util.spec_from_file_location(
        'mos_coordinator_standalone_vision_adapter',
        module_path,
    )
    if spec is None or spec.loader is None:
        raise ImportError(f'无法加载 VisionAdapter: {module_path}')

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.VisionAdapter


VisionAdapter = _load_vision_adapter()


class VisionAdapterRealTest(Node):
    """不依赖导航和机械臂的 VisionAdapter 真机测试节点。"""

    def __init__(self, side: str = 'left', auto_request: bool = True):
        super().__init__('vision_adapter_real_test')

        if side not in {'left', 'right'}:
            raise ValueError(f'不支持的相机侧: {side}')

        self.side = side
        self.vision = VisionAdapter(self)
        self._last_result = object()

        self.create_service(
            Trigger,
            '/vision_adapter_real_test/request_left',
            self._request_left,
        )
        self.create_service(
            Trigger,
            '/vision_adapter_real_test/request_right',
            self._request_right,
        )

        self._result_timer = self.create_timer(
            0.5,
            self._report_result,
        )
        self._auto_request_timer = None
        if auto_request:
            self._auto_request_timer = self.create_timer(
                1.0,
                self._auto_request,
            )

        self.get_logger().info(
            'VisionAdapter 独立测试节点已启动：'
            f'side={self.side}, auto_request={auto_request}'
        )
        self.get_logger().info(
            '不包含 NavigationAdapter、ArmAdapter、FetchAdapter'
        )

    def _auto_request(self):
        if self._auto_request_timer is not None:
            self._auto_request_timer.cancel()
            self._auto_request_timer = None
        self._request_side(self.side)

    def _request_left(self, _request, response):
        self._request_side('left')
        response.success = True
        response.message = '已发布 left_start'
        return response

    def _request_right(self, _request, response):
        self._request_side('right')
        response.success = True
        response.message = '已发布 right_start'
        return response

    def _request_side(self, side: str):
        self.vision.request_fullness_check(side)
        self.get_logger().info(
            f'已请求 {side} 腕部相机满载检测，等待 route_signal/status'
        )

    def _report_result(self):
        result = self.vision.cart_result()
        if result == self._last_result:
            return

        self._last_result = result
        self.get_logger().info(
            f'VisionAdapter cart_result = {result!r}'
        )

    # VisionAdapter 需要宿主节点提供这些回调；本测试只记录，不执行动作。
    def on_cart_3d_detected(self, _cart_pose, class_id: int):
        self.get_logger().debug(f'收到 3D 小车检测 class_id={class_id}')

    def on_handle_positions_received(
        self,
        _left_handle,
        _right_handle,
        task_name: str,
    ):
        self.get_logger().info(
            f'收到抓取点 {task_name}（测试节点不执行抓取）'
        )

    def on_clearance_received(self, is_clear: bool):
        self.get_logger().info(
            f'收到视觉离开信号: {is_clear}（测试节点不执行导航）'
        )


def _parse_cli(args):
    parser = argparse.ArgumentParser(
        description='单独测试 VisionAdapter，不启动导航和机械臂',
    )
    parser.add_argument(
        '--side',
        choices=('left', 'right'),
        default='left',
        help='自动请求的腕部相机，默认 left',
    )
    parser.add_argument(
        '--auto-request',
        dest='auto_request',
        action='store_true',
        default=True,
        help='启动后自动请求一次检测（默认开启）',
    )
    parser.add_argument(
        '--no-auto-request',
        dest='auto_request',
        action='store_false',
        help='启动后不自动请求，通过 Trigger 服务手动触发',
    )
    parsed, ros_args = parser.parse_known_args(args)
    return parsed, ros_args


def main(args=None):
    cli, ros_args = _parse_cli(sys.argv[1:] if args is None else args)
    rclpy.init(args=ros_args)
    node = VisionAdapterRealTest(cli.side, cli.auto_request)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断，停止 VisionAdapter 测试')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
