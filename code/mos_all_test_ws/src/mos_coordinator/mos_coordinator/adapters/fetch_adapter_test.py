#!/usr/bin/env python3
"""单独测试 FetchAdapter：用 stub 子系统驱动阻塞 fetch()。

覆盖：
  1. 路线 B：把手距离合适，直接抓取
  2. 路线 A：过远 → 底盘对准 → 再检测 → 抓取
  3. 把手检测超时 → 重试后失败
  4. 机械臂抓取失败 → 重试后失败

运行（需已 source ROS 工作空间）::

    cd /root/hanjiatong/mos_coordinator
    python3 mos_coordinator/adapters/fetch_adapter_test.py
"""

from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from mos_coordinator.adapters.fetch_adapter import FetchAdapter


def make_handle(x: float, y: float, z: float) -> PointStamped:
    point = PointStamped()
    point.header.frame_id = 'body_link'
    point.point.x = x
    point.point.y = y
    point.point.z = z
    return point


class StubVision:
    """可按调用次序返回不同把手距离。"""

    def __init__(self, node: Node, distances):
        self.node = node
        self.distances = list(distances)
        self.call_count = 0

    def reset_selector(self):
        pass

    def set_vision_enabled(self, enabled: bool):
        pass

    def set_selector_enabled(self, enabled: bool):
        pass

    def request_handle_detection(self):
        idx = min(self.call_count, len(self.distances) - 1)
        distance_x = self.distances[idx]
        self.call_count += 1

        def _inject():
            time.sleep(0.05)
            left = make_handle(distance_x, 0.3, -0.2)
            right = make_handle(distance_x, -0.3, -0.2)
            self.node.on_handle_positions_received(left, right, 'test')

        threading.Thread(target=_inject, daemon=True).start()


class SilentVision(StubVision):
    """不注入把手结果，用于制造检测超时。"""

    def request_handle_detection(self):
        self.call_count += 1
        self.node.get_logger().info('[stub] 故意不返回把手检测结果')


class StubNavigation:
    def __init__(self, node: Node, success: bool = True):
        self.node = node
        self.success = success
        self.align_calls = []

    def request_alignment(self, target_x=None, return_to_origin: bool = False) -> bool:
        self.align_calls.append(
            {'target_x': target_x, 'return_to_origin': return_to_origin}
        )

        def _done():
            time.sleep(0.05)
            self.node.on_alignment_complete(self.success)

        threading.Thread(target=_done, daemon=True).start()
        return True


class StubArm:
    def __init__(self, node: Node, success: bool = True):
        self.node = node
        self.success = success
        self.grasp_calls = 0

    def grasp_at_point(self, left: PointStamped, right: PointStamped):
        self.grasp_calls += 1

        def _done():
            # 延后回调，确保业务侧已 transition_to(GRASPING)
            time.sleep(0.05)
            self.node.on_grasp_result(self.success)

        threading.Thread(target=_done, daemon=True).start()


class FetchTestNode(Node):
    def __init__(
        self,
        *,
        distances=(0.6,),
        arm_success: bool = True,
        alignment_success: bool = True,
        silent_vision: bool = False,
        max_retries: int = 2,
        detection_timeout_sec: float = 1.0,
        alignment_timeout_sec: float = 2.0,
        grasp_timeout_sec: float = 2.0,
    ):
        super().__init__('fetch_adapter_test')

        if silent_vision:
            self.vision = SilentVision(self, distances=distances)
        else:
            self.vision = StubVision(self, distances=distances)

        self.navigation = StubNavigation(self, success=alignment_success)
        self.arm = StubArm(self, success=arm_success)
        self.fetch = FetchAdapter(
            self,
            vision=self.vision,
            navigation=self.navigation,
            arm=self.arm,
            max_retries=max_retries,
            detection_timeout_sec=detection_timeout_sec,
            alignment_timeout_sec=alignment_timeout_sec,
            grasp_timeout_sec=grasp_timeout_sec,
        )

    def on_handle_positions_received(self, left, right, task_name: str = 'grasp'):
        self.fetch.notify_handle_positions(left, right, task_name)

    def on_alignment_complete(self, success: bool = True):
        self.fetch.notify_alignment_complete(success)

    def on_grasp_result(self, success: bool):
        self.fetch.notify_grasp_result(success)


def run_case(name: str, **kwargs):
    print(f'\n===== {name} =====')
    node = FetchTestNode(**kwargs)
    ok = node.fetch.fetch()
    summary = {
        'ok': ok,
        'did_alignment': node.fetch.did_alignment,
        'align_calls': len(node.navigation.align_calls),
        'grasp_calls': node.arm.grasp_calls,
        'vision_calls': node.vision.call_count,
        'error': node.fetch.last_error_msg,
    }
    print(
        f"result={summary['ok']}, did_alignment={summary['did_alignment']}, "
        f"align_calls={summary['align_calls']}, "
        f"grasp_calls={summary['grasp_calls']}, "
        f"vision_calls={summary['vision_calls']}, "
        f"error={summary['error']!r}"
    )
    node.destroy_node()
    return summary


def main():
    rclpy.init()
    failures = []

    # 1) 路线 B：距离合适，直接抓取
    s = run_case('direct_grasp', distances=(0.6,))
    if not s['ok']:
        failures.append('direct_grasp should succeed')
    if s['did_alignment']:
        failures.append('direct_grasp should not align')
    if s['grasp_calls'] < 1:
        failures.append('direct_grasp should grasp once')

    # 2) 路线 A：先远后近（对准后再检测进入抓取）
    s = run_case('align_then_grasp', distances=(1.2, 0.6))
    if not s['ok']:
        failures.append('align_then_grasp should succeed')
    if not s['did_alignment']:
        failures.append('align_then_grasp should set did_alignment')
    if s['align_calls'] < 1:
        failures.append('align_then_grasp should request alignment')
    if s['grasp_calls'] < 1:
        failures.append('align_then_grasp should grasp')
    if s['vision_calls'] < 2:
        failures.append('align_then_grasp should re-detect after alignment')

    # 3) 把手检测超时 → 最终失败
    s = run_case(
        'detection_timeout',
        silent_vision=True,
        max_retries=2,
        detection_timeout_sec=0.3,
    )
    if s['ok']:
        failures.append('detection_timeout should fail')

    # 4) 抓取失败 → 最终失败
    s = run_case(
        'grasp_fail',
        distances=(0.6,),
        arm_success=False,
        max_retries=2,
    )
    if s['ok']:
        failures.append('grasp_fail should fail')

    rclpy.shutdown()

    if failures:
        print('\nFAILED:')
        for item in failures:
            print(f'  - {item}')
        raise SystemExit(1)

    print('\nALL PASSED')


if __name__ == '__main__':
    main()
