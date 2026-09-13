#!/usr/bin/env python3
"""阻塞式抓取流程模块。

将已验证的 on_alignment_complete / on_handle_positions_received 业务逻辑
整理为独立模块，由 fetch() 以阻塞、可重试方式驱动完整抓取流程：

  把手检测(POSITIONING) → [可选底盘对准(ALIGNING)] → 机械臂抓取(GRASPING)
"""

from __future__ import annotations

import threading
import time

from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from mos_coordinator.states import TaskState


class FetchAdapter:
    """抓取流程适配器：核心业务逻辑 + 阻塞可重试入口。"""

    ALIGNMENT_THRESHOLD = 0.8579  # 对准阈值（与 dummy_chassis 一致）

    def __init__(
        self,
        node: Node,
        vision,
        navigation,
        arm=None,
        max_retries: int = 3,
        detection_timeout_sec: float = 30.0,
        alignment_timeout_sec: float = 60.0,
        grasp_timeout_sec: float = 60.0,
    ):
        self.node = node
        self.vision = vision
        self.navigation = navigation
        self.arm = arm

        self.max_retries = max_retries
        self.detection_timeout_sec = detection_timeout_sec
        self.alignment_timeout_sec = alignment_timeout_sec
        self.grasp_timeout_sec = grasp_timeout_sec

        # ===== 流程内部状态（与原 coordinator 字段对齐） =====
        self.state = TaskState.POSITIONING
        self.previous_state = None
        self.last_handle_left = None
        self.last_handle_right = None
        self.last_error_msg = ""
        self.detection_timeout_timer = None

        self.delivery_point = getattr(node, 'delivery_point', None)

        # ===== 阻塞等待同步原语 =====
        self._active = False
        self._lock = threading.Lock()
        self._handles_event = threading.Event()
        self._alignment_event = threading.Event()
        self._grasp_event = threading.Event()
        self._pending_left = None
        self._pending_right = None
        self._pending_task_name = 'grasp'
        self._alignment_ok = False
        self._grasp_success = False
        self._detection_deadline = None

        # ===== 后台线程支持 =====
        self._fetch_thread = None
        self._fetch_result = None

        self.node.get_logger().info('FetchAdapter initialized')

    def fetch(self) -> bool:
        """阻塞执行抓取流程，失败自动重试。

        Returns:
            bool: 抓取成功为 True，超过重试次数仍失败为 False
        """
        for attempt in range(1, self.max_retries + 1):
            self.node.get_logger().info(
                f'Fetch 开始第 {attempt}/{self.max_retries} 次尝试'
            )
            if self._fetch_once():
                self.node.get_logger().info('Fetch 成功')
                return True
            self.node.get_logger().warn(
                f'Fetch 第 {attempt}/{self.max_retries} 次失败: {self.last_error_msg}'
            )
            time.sleep(0.5)

        self.node.get_logger().error(
            f'Fetch 失败，已重试 {self.max_retries} 次'
        )
        return False

    def fetch_async(self):
        """非阻塞版本：在后台线程执行抓取流程
        
        立即返回，抓取流程在后台线程运行。
        使用 is_fetch_running() 检查状态，get_fetch_result() 获取结果。
        """
        if self._fetch_thread is not None and self._fetch_thread.is_alive():
            self.node.get_logger().warn('Fetch 已在运行中，忽略重复调用')
            return
        
        self._fetch_result = None
        self._fetch_thread = threading.Thread(
            target=self._fetch_with_result,
            daemon=True,
            name='FetchAdapter'
        )
        self._fetch_thread.start()
        self.node.get_logger().info('🚀 Fetch 后台线程已启动')

    def _fetch_with_result(self):
        """内部：执行fetch并保存结果"""
        try:
            self._fetch_result = self.fetch()
        except Exception as e:
            self.node.get_logger().error(f'Fetch 后台线程异常: {e}')
            self._fetch_result = False

    def is_fetch_running(self) -> bool:
        """检查抓取流程是否仍在运行"""
        return self._fetch_thread is not None and self._fetch_thread.is_alive()

    def get_fetch_result(self) -> bool | None:
        """获取抓取结果
        
        Returns:
            True: 抓取成功
            False: 抓取失败
            None: 尚未完成
        """
        if self.is_fetch_running():
            return None
        return self._fetch_result

    def wait_fetch_completion(self, timeout: float = None) -> bool | None:
        """等待抓取流程完成
        
        Args:
            timeout: 等待超时（秒），None表示无限等待
            
        Returns:
            True: 抓取成功
            False: 抓取失败
            None: 超时
        """
        if self._fetch_thread is None:
            return self._fetch_result
        
        self._fetch_thread.join(timeout=timeout)
        
        if self.is_fetch_running():
            return None  # 超时
        return self._fetch_result

    def _fetch_once(self) -> bool:
        """单次抓取尝试（阻塞）。"""
        self._reset_attempt_state()
        self._active = True

        try:
            # 启动视觉系统并开始检测（对齐原 on_waypoint_reached / POSITIONING）
            self.vision.reset_selector()
            self.vision.set_vision_enabled(True)
            self.vision.set_selector_enabled(True)
            self.transition_to(TaskState.POSITIONING)
            self.vision.request_handle_detection()
            self._start_detection_timeout()

            while self._active:
                if self.state == TaskState.POSITIONING:
                    if not self._wait_for_handles():
                        self.last_error_msg = (
                            f'把手检测超时（{self.detection_timeout_sec}秒）'
                        )
                        return False

                    left = self._pending_left
                    right = self._pending_right
                    task_name = self._pending_task_name
                    self.on_handle_positions_received(left, right, task_name)
                    if self.state == TaskState.ERROR:
                        return False

                elif self.state == TaskState.GRASPING:
                    if not self._wait_for_grasp():
                        self.last_error_msg = (
                            self.last_error_msg
                            or f'机械臂抓取超时（{self.grasp_timeout_sec}秒）'
                        )
                        return False

                    if self._grasp_success:
                        self.node.get_logger().info('✅ 抓取成功（含自动前进和后退）')
                        return True

                    self.last_error_msg = '机械臂抓取失败'
                    return False

                elif self.state == TaskState.ERROR:
                    return False

                else:
                    self.last_error_msg = f'意外状态: {self.state.value}'
                    return False

            self.last_error_msg = self.last_error_msg or 'Fetch 被中止'
            return False
        finally:
            self._active = False
            self._cancel_detection_timeout()
            try:
                self.vision.set_vision_enabled(False)
                self.vision.set_selector_enabled(False)
            except Exception as e:
                self.node.get_logger().warn(f'关闭视觉系统时出错: {e}')

    def _reset_attempt_state(self):
        self.last_error_msg = ""
        self.last_handle_left = None
        self.last_handle_right = None
        self._pending_left = None
        self._pending_right = None
        self._pending_task_name = 'grasp'
        self._alignment_ok = False
        self._grasp_success = False
        self._handles_event.clear()
        self._alignment_event.clear()
        self._grasp_event.clear()

    # ===== 外部事件通知（由 vision / navigation / arm 回调转入） =====

    def notify_handle_positions(
        self,
        left_handle: PointStamped,
        right_handle: PointStamped,
        task_name: str = 'grasp',
    ):
        """视觉把手检测结果就绪。"""
        if not self._active or self.state != TaskState.POSITIONING:
            return
        with self._lock:
            self._pending_left = left_handle
            self._pending_right = right_handle
            self._pending_task_name = task_name
        self._handles_event.set()

    def notify_alignment_complete(self, success: bool = True):
        """底盘对准服务完成。"""
        if not self._active:
            return
        self._alignment_ok = success
        if not success:
            self.last_error_msg = '底盘对准失败'
        self._alignment_event.set()

    def notify_grasp_result(self, success: bool):
        """机械臂抓取结果就绪。"""
        if not self._active:
            return
        if self.state != TaskState.GRASPING:
            return
        self._grasp_success = success
        if not success:
            self.last_error_msg = '机械臂抓取失败'
        self._grasp_event.set()

    # ===== 已验证业务逻辑（尽量完整保留，仅将 get_logger 改为 node.get_logger） =====

    def on_alignment_complete(self):
        """底盘对准完成事件（保留用于兼容性，新架构下不再使用）"""
        pass

    def on_handle_positions_received(
        self,
        left_handle: PointStamped,
        right_handle: PointStamped,
        task_name: str = 'grasp',
    ):
        """接收把手位置信息：直接调用机械臂抓取（机械臂内部处理前进/后退）

        Args:
            left_handle: 左手把手位置
            right_handle: 右手把手位置
            task_name: 任务名称
        """
        if self.state != TaskState.POSITIONING:
            return

        # 取消超时定时器（安全标识：检测成功）
        self._cancel_detection_timeout()

        # 保存把手位置（用于重试）
        self.last_handle_left = left_handle
        self.last_handle_right = right_handle

        self.node.get_logger().info(
            f"收到把手位置 - 左手: ({left_handle.point.x:.3f}, {left_handle.point.y:.3f}, {left_handle.point.z:.3f}), "
            f"右手: ({right_handle.point.x:.3f}, {right_handle.point.y:.3f}, {right_handle.point.z:.3f})"
        )

        # 关闭视觉系统（安全标识：抓取时不再检测）
        self.vision.set_vision_enabled(False)
        self.vision.set_selector_enabled(False)

        # 直接抓取（机械臂服务内部会根据把手位置自动前进/后退）
        self.node.get_logger().info("✅ 开始抓取（机械臂将自动处理底盘前进和后退）")
        self.transition_to(TaskState.GRASPING)
        self.arm.grasp_at_point(left_handle, right_handle)

    def transition_to(self, new_state: TaskState, error_msg: str = ""):
        """状态转换（支持错误消息）"""
        self.previous_state = self.state
        self.state = new_state
        self.node.get_logger().info(
            f"Fetch 状态转换: {self.previous_state.value} -> {self.state.value}"
        )

        if new_state == TaskState.ERROR:
            self.last_error_msg = error_msg or self.last_error_msg

        if new_state == TaskState.POSITIONING:
            self._handles_event.clear()
            self._pending_left = None
            self._pending_right = None
        elif new_state == TaskState.ALIGNING:
            self._alignment_event.clear()
            self._alignment_ok = False
        elif new_state == TaskState.GRASPING:
            self._grasp_event.clear()
            self._grasp_success = False

    # ===== 超时 / 阻塞等待 =====

    def _start_detection_timeout(self):
        """启动把手检测超时定时器（安全标识）"""
        self._cancel_detection_timeout()
        self._detection_deadline = time.monotonic() + self.detection_timeout_sec
        self.node.get_logger().info(
            f'启动检测超时保护: {self.detection_timeout_sec}秒'
        )

    def _cancel_detection_timeout(self):
        """取消把手检测超时定时器"""
        self._detection_deadline = None
        if self.detection_timeout_timer is not None:
            try:
                self.detection_timeout_timer.cancel()
            except Exception:
                pass
            self.detection_timeout_timer = None

    def _wait_for_handles(self) -> bool:
        deadline = getattr(self, '_detection_deadline', None)
        if deadline is None:
            deadline = time.monotonic() + self.detection_timeout_sec
        return self._wait_event(self._handles_event, deadline)

    def _wait_for_alignment(self) -> bool:
        deadline = time.monotonic() + self.alignment_timeout_sec
        if not self._wait_event(self._alignment_event, deadline):
            return False
        return self._alignment_ok

    def _wait_for_grasp(self) -> bool:
        deadline = time.monotonic() + self.grasp_timeout_sec
        return self._wait_event(self._grasp_event, deadline)

    def _wait_event(self, event: threading.Event, deadline: float) -> bool:
        """阻塞等待事件，直到置位或超时。"""
        while not event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            event.wait(timeout=min(0.1, remaining))
        return True
