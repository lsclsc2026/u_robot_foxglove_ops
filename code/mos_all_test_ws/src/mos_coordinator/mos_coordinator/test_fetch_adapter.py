#!/usr/bin/env python3
"""单独测试 FetchAdapter：用 stub 子系统驱动阻塞 fetch()。

不依赖真实的视觉/导航/机械臂节点，所有子系统都用 stub 模拟，
通过线程延迟注入回调来模拟真实异步行为。

覆盖测试场景：
  1. direct_grasp  : 路线 B - 把手距离合适，直接抓取（成功）
  2. align_then_grasp : 路线 A - 把手太远 → 底盘对准 → 再检测 → 抓取（成功）
  3. detection_timeout : 把手检测超时 → 重试后失败
  4. grasp_fail    : 机械臂抓取失败 → 重试后失败
  5. interactive   : 交互模式，手动控制每个事件（用于调试）

运行方式（需先 source 工作空间）::

    # 方式 1: 运行所有测试场景
    python3 mos_coordinator/test_fetch_adapter.py

    # 方式 2: 只跑某个场景
    python3 mos_coordinator/test_fetch_adapter.py --case direct_grasp

    # 方式 3: 交互模式（手动触发事件）
    python3 mos_coordinator/test_fetch_adapter.py --case interactive
"""

from __future__ import annotations

import argparse
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from mos_coordinator.adapters.fetch_adapter import FetchAdapter


# ===== 辅助函数 =====

def make_handle(x: float, y: float, z: float) -> PointStamped:
    """构造把手位置点。"""
    point = PointStamped()
    point.header.frame_id = 'body_link'
    point.point.x = x
    point.point.y = y
    point.point.z = z
    return point


# ===== Stub 子系统 =====

class StubVision:
    """Stub 视觉子系统：按调用次序返回不同把手距离。

    distances 是一个列表，每次 request_handle_detection 消耗一个距离值。
    通过线程延迟注入把手位置，模拟真实异步回调。
    """

    def __init__(self, node: Node, distances, silent: bool = False):
        self.node = node
        self.distances = list(distances)
        self.call_count = 0
        self.silent = silent  # True = 不返回结果（用于测试超时）

    def reset_selector(self):
        self.node.get_logger().info('[stub vision] reset_selector')

    def set_vision_enabled(self, enabled: bool):
        self.node.get_logger().info(f'[stub vision] set_vision_enabled={enabled}')

    def set_selector_enabled(self, enabled: bool):
        self.node.get_logger().info(f'[stub vision] set_selector_enabled={enabled}')

    def request_handle_detection(self):
        self.call_count += 1
        if self.silent:
            self.node.get_logger().info('[stub vision] request_handle_detection (silent, 不返回)')
            return

        idx = min(self.call_count - 1, len(self.distances) - 1)
        distance_x = self.distances[idx]
        self.node.get_logger().info(
            f'[stub vision] request_handle_detection #{self.call_count}, 将返回 x={distance_x}'
        )

        def _inject():
            time.sleep(0.1)  # 模拟检测延迟
            left = make_handle(distance_x, 0.18, -0.55)
            right = make_handle(distance_x, -0.18, 0.55)
            self.node.on_handle_positions_received(left, right, 'test')

        threading.Thread(target=_inject, daemon=True).start()


class StubNavigation:
    """Stub 导航子系统：模拟底盘对准和导航。

    align_chassis() 返回 True 表示请求发送成功，
    通过线程延迟调用 on_alignment_complete 模拟对准完成回调。
    """

    def __init__(self, node: Node, success: bool = True):
        self.node = node
        self.success = success
        self.align_calls = 0
        self.navigate_calls = 0

    def align_chassis(self) -> bool:
        self.align_calls += 1
        self.node.get_logger().info(f'[stub nav] align_chassis #{self.align_calls}')

        def _done():
            time.sleep(0.1)  # 模拟对准延迟
            self.node.on_alignment_complete(self.success)

        threading.Thread(target=_done, daemon=True).start()
        return True

    def navigate_to_pose(self, pose):
        self.navigate_calls += 1
        self.node.get_logger().info(f'[stub nav] navigate_to_pose #{self.navigate_calls}')


class StubArm:
    """Stub 机械臂子系统：模拟抓取。

    grasp_at_point() 通过线程延迟调用 on_grasp_result 模拟抓取完成回调。
    """

    def __init__(self, node: Node, success: bool = True):
        self.node = node
        self.success = success
        self.grasp_calls = 0

    def grasp_at_point(self, left: PointStamped, right: PointStamped):
        self.grasp_calls += 1
        self.node.get_logger().info(
            f'[stub arm] grasp_at_point #{self.grasp_calls} '
            f'left=({left.point.x:.2f},{left.point.y:.2f},{left.point.z:.2f}) '
            f'right=({right.point.x:.2f},{right.point.y:.2f},{right.point.z:.2f})'
        )

        def _done():
            time.sleep(0.1)  # 模拟抓取延迟
            self.node.on_grasp_result(self.success)

        threading.Thread(target=_done, daemon=True).start()


# ===== 测试节点 =====

class FetchTestNode(Node):
    """测试节点：持有 FetchAdapter 和 stub 子系统，转发回调事件。"""

    def __init__(
        self,
        *,
        distances=(0.6,),
        arm_success: bool = True,
        alignment_success: bool = True,
        silent_vision: bool = False,
        max_retries: int = 2,
        detection_timeout_sec: float = 2.0,
        alignment_timeout_sec: float = 2.0,
        grasp_timeout_sec: float = 2.0,
    ):
        super().__init__('test_fetch_adapter')

        self.vision = StubVision(self, distances=distances, silent=silent_vision)
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

    # ===== 回调转发（FetchAdapter 期望 node 上有这些方法） =====

    def on_handle_positions_received(self, left, right, task_name: str = 'grasp'):
        """视觉把手检测结果 → 转发给 FetchAdapter。"""
        self.fetch.notify_handle_positions(left, right, task_name)

    def on_alignment_complete(self, success: bool = True):
        """底盘对准完成 → 转发给 FetchAdapter。"""
        self.fetch.notify_alignment_complete(success)

    def on_grasp_result(self, success: bool):
        """机械臂抓取结果 → 转发给 FetchAdapter。"""
        self.fetch.notify_grasp_result(success)


# ===== 测试场景 =====

def run_case(name: str, **kwargs) -> dict:
    """运行单个测试场景并返回结果摘要。"""
    print(f'\n{"=" * 60}')
    print(f'测试场景: {name}')
    print(f'{"=" * 60}')

    node = FetchTestNode(**kwargs)
    ok = node.fetch.fetch()

    summary = {
        'ok': ok,
        'did_alignment': node.fetch.did_alignment,
        'align_calls': node.navigation.align_calls,
        'grasp_calls': node.arm.grasp_calls,
        'vision_calls': node.vision.call_count,
        'error': node.fetch.last_error_msg,
    }
    print(
        f"\n结果: success={summary['ok']}, "
        f"did_alignment={summary['did_alignment']}, "
        f"align_calls={summary['align_calls']}, "
        f"grasp_calls={summary['grasp_calls']}, "
        f"vision_calls={summary['vision_calls']}, "
        f"error={summary['error']!r}"
    )
    node.destroy_node()
    return summary


def case_direct_grasp() -> list:
    """场景 1: 路线 B - 距离合适，直接抓取。"""
    s = run_case(
        'direct_grasp (距离 0.6m < 0.8579m, 直接抓取)',
        distances=(0.6,),
        max_retries=1,
    )
    failures = []
    if not s['ok']:
        failures.append('direct_grasp 应该成功')
    if s['did_alignment']:
        failures.append('direct_grasp 不应该对准')
    if s['grasp_calls'] != 1:
        failures.append(f"direct_grasp 应该抓取 1 次, 实际 {s['grasp_calls']} 次")
    return failures


def case_align_then_grasp() -> list:
    """场景 2: 路线 A - 先远后近（对准后再检测进入抓取）。"""
    s = run_case(
        'align_then_grasp (距离 1.2m > 0.8579m → 对准 → 距离 0.6m → 抓取)',
        distances=(1.2, 0.6),
        max_retries=1,
    )
    failures = []
    if not s['ok']:
        failures.append('align_then_grasp 应该成功')
    if not s['did_alignment']:
        failures.append('align_then_grasp 应执行底盘对准')
    if s['align_calls'] < 1:
        failures.append('align_then_grasp 应请求对准')
    if s['grasp_calls'] < 1:
        failures.append('align_then_grasp 应执行抓取')
    if s['vision_calls'] < 2:
        failures.append('align_then_grasp 对准后应重新检测')
    return failures


def case_detection_timeout() -> list:
    """场景 3: 把手检测超时 → 最终失败。"""
    s = run_case(
        'detection_timeout (视觉不返回 → 检测超时 → 重试后失败)',
        silent_vision=True,
        max_retries=2,
        detection_timeout_sec=0.3,
    )
    failures = []
    if s['ok']:
        failures.append('detection_timeout 应该失败')
    return failures


def case_grasp_fail() -> list:
    """场景 4: 机械臂抓取失败 → 重试后失败。"""
    s = run_case(
        'grasp_fail (机械臂返回失败 → 重试后失败)',
        distances=(0.6,),
        arm_success=False,
        max_retries=2,
    )
    failures = []
    if s['ok']:
        failures.append('grasp_fail 应该失败')
    return failures


def case_interactive():
    """场景 5: 交互模式 - 手动控制每个事件，用于调试。"""
    print(f'\n{"=" * 60}')
    print('交互模式: 手动触发事件')
    print(f'{"=" * 60}')

    node = FetchTestNode(
        distances=(0.6,),
        max_retries=1,
        detection_timeout_sec=999.0,  # 关闭超时
        alignment_timeout_sec=999.0,
        grasp_timeout_sec=999.0,
    )
    node.vision.silent = True  # 不自动返回结果

    print('\n在另一个线程启动 fetch()，你手动触发事件...')
    print('命令:')
    print('  h      - 注入把手位置 (x=0.6, 距离合适)')
    print('  h-far  - 注入把手位置 (x=1.2, 距离太远 → 需要对准)')
    print('  a      - 通知底盘对准完成 (成功)')
    print('  a-fail - 通知底盘对准完成 (失败)')
    print('  g      - 通知机械臂抓取完成 (成功)')
    print('  g-fail - 通知机械臂抓取完成 (失败)')
    print('  s      - 查看当前状态')
    print('  q      - 退出')

    result = {'ok': None}

    def _fetch_thread():
        result['ok'] = node.fetch.fetch()

    t = threading.Thread(target=_fetch_thread, daemon=True)
    t.start()

    while t.is_alive():
        try:
            cmd = input('\n> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == 'h':
            left = make_handle(0.6, 0.18, -0.55)
            right = make_handle(0.6, -0.18, 0.55)
            node.on_handle_positions_received(left, right, 'test')
            print('已注入把手位置 (x=0.6)')
        elif cmd == 'h-far':
            left = make_handle(1.2, 0.18, -0.55)
            right = make_handle(1.2, -0.18, 0.55)
            node.on_handle_positions_received(left, right, 'test')
            print('已注入把手位置 (x=1.2)')
        elif cmd == 'a':
            node.on_alignment_complete(True)
            print('已通知对准成功')
        elif cmd == 'a-fail':
            node.on_alignment_complete(False)
            print('已通知对准失败')
        elif cmd == 'g':
            node.on_grasp_result(True)
            print('已通知抓取成功')
        elif cmd == 'g-fail':
            node.on_grasp_result(False)
            print('已通知抓取失败')
        elif cmd == 's':
            print(f'当前状态: {node.fetch.state.value}')
            print(f'did_alignment: {node.fetch.did_alignment}')
            print(f'vision_calls: {node.vision.call_count}')
            print(f'align_calls: {node.navigation.align_calls}')
            print(f'grasp_calls: {node.arm.grasp_calls}')
        elif cmd == 'q':
            print('退出')
            break

    t.join(timeout=1.0)
    print(f'\nfetch 结果: {result["ok"]}')
    node.destroy_node()
    return []


# ===== 主入口 =====

def main():
    parser = argparse.ArgumentParser(description='FetchAdapter 单独测试脚本')
    parser.add_argument(
        '--case',
        default='all',
        choices=['all', 'direct_grasp', 'align_then_grasp',
                 'detection_timeout', 'grasp_fail', 'interactive'],
        help='选择测试场景 (默认: all)',
    )
    args = parser.parse_args()

    rclpy.init()
    all_failures = []

    try:
        if args.case == 'all':
            all_failures += case_direct_grasp()
            all_failures += case_align_then_grasp()
            all_failures += case_detection_timeout()
            all_failures += case_grasp_fail()
        elif args.case == 'direct_grasp':
            all_failures += case_direct_grasp()
        elif args.case == 'align_then_grasp':
            all_failures += case_align_then_grasp()
        elif args.case == 'detection_timeout':
            all_failures += case_detection_timeout()
        elif args.case == 'grasp_fail':
            all_failures += case_grasp_fail()
        elif args.case == 'interactive':
            case_interactive()
    finally:
        rclpy.shutdown()

    # 汇总结果
    if args.case != 'interactive':
        print(f'\n{"=" * 60}')
        if all_failures:
            print('测试失败:')
            for item in all_failures:
                print(f'  - {item}')
            raise SystemExit(1)
        else:
            print('所有测试通过 ✅')
        print(f'{"=" * 60}')


if __name__ == '__main__':
    main()
