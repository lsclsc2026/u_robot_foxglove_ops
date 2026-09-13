from __future__ import annotations

import argparse
import copy
import json
import os
import subprocess
import sys
import time
from contextlib import nullcontext
from pathlib import Path
from typing import Any

from .alarm import alarm_to_json, make_alarm
from .config import load_yaml, project_root, runtime_path
from .storage import (
    append_rotating_jsonl,
    atomic_write_json,
    exclusive_lock,
    now_iso,
    read_json,
)


LEVEL_RANK = {"NORMAL": 0, "P2": 1, "P1": 2, "P0": 3}
VALID_SCOPES = ("navigation", "base", "arm", "vision", "scheduler")
STATE_PATH = "latest_protection.json"


def default_state() -> dict[str, Any]:
    return {
        "schema_version": 1,
        "state": "NORMAL",
        "active": False,
        "event_count": 0,
        "isolated_subsystems": [],
        "stopped_subsystems": [],
        "protection_complete": True,
        "hardware_estop_confirmed": False,
        "updated_at": now_iso(),
    }


def load_state() -> dict[str, Any]:
    state = read_json(project_root() / "runtime" / STATE_PATH, default_state())
    return state if isinstance(state, dict) else default_state()


def _validate_action(action: Any, label: str) -> dict[str, Any]:
    if not isinstance(action, dict):
        raise RuntimeError(f"动作 {label} 必须是映射")
    argv = action.get("argv", [])
    if not isinstance(argv, list) or not all(
        isinstance(item, str) and item for item in argv
    ):
        raise RuntimeError(f"动作 {label}.argv 必须是非空字符串数组")
    return action


def run_action(
    label: str,
    action: dict[str, Any],
    *,
    timeout: float,
    dry_run: bool = False,
) -> dict[str, Any]:
    action = _validate_action(action, label)
    enabled = bool(action.get("enabled", False))
    argv = action.get("argv", [])
    result: dict[str, Any] = {
        "action": label,
        "description": str(action.get("description", "")),
        "enabled": enabled,
        "argv": argv,
        "started_at": now_iso(),
    }
    if not enabled:
        result.update(status="disabled", success=False)
        return result
    if not argv:
        result.update(status="invalid", success=False, error="已启用动作没有 argv")
        return result
    if dry_run:
        result.update(status="dry_run", success=True)
        return result
    started = time.monotonic()
    try:
        completed = subprocess.run(
            argv,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
            env=os.environ.copy(),
        )
        result.update(
            status="success" if completed.returncode == 0 else "failed",
            success=completed.returncode == 0,
            returncode=completed.returncode,
            stdout=completed.stdout.strip()[-4000:],
            stderr=completed.stderr.strip()[-4000:],
        )
    except subprocess.TimeoutExpired:
        result.update(status="timeout", success=False, error=f"超过 {timeout:g} 秒")
    except OSError as exc:
        result.update(status="error", success=False, error=str(exc))
    result["elapsed_seconds"] = round(time.monotonic() - started, 3)
    return result


def _action_from_config(config: dict[str, Any], name: str) -> dict[str, Any]:
    actions = config.get("actions", {})
    if not isinstance(actions, dict) or name not in actions:
        return {
            "enabled": False,
            "argv": [],
            "description": f"配置中缺少动作 {name}",
        }
    return _validate_action(actions[name], name)


def _publish_alarm_best_effort(alarm: dict[str, Any]) -> str | None:
    try:
        import rclpy
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from std_msgs.msg import String

        was_initialized = rclpy.ok()
        if not was_initialized:
            rclpy.init(args=None)
        node = rclpy.create_node(f"mos_protection_event_{os.getpid()}")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        publisher = node.create_publisher(String, "/mos/ops/alarm_input", qos)
        message = String()
        message.data = alarm_to_json(alarm)
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.15)
        node.destroy_node()
        if not was_initialized and rclpy.ok():
            rclpy.shutdown()
        return None
    except Exception as exc:  # ROS 告警失败不能阻止本地保护。
        return str(exc)


def _save_event(config: dict[str, Any], event: dict[str, Any]) -> None:
    log_config = config.get("event_log", {})
    append_rotating_jsonl(
        runtime_path("protection_events.jsonl"),
        event,
        max_bytes=int(log_config.get("max_bytes", 5 * 1024 * 1024)),
        backups=int(log_config.get("backups", 3)),
    )


def _record_recovery_failure(
    config: dict[str, Any],
    previous: dict[str, Any],
    *,
    operator: str,
    onsite_confirmed: bool,
    actions: list[dict[str, Any]],
    reason: str,
    remaining_stopped: set[str],
    dry_run: bool = False,
) -> None:
    if dry_run:
        return
    timestamp = now_iso()
    state = copy.deepcopy(previous)
    state.update(
        updated_at=timestamp,
        last_recovery_attempt_at=timestamp,
        last_recovery_operator=operator,
        last_recovery_error=reason,
        stopped_subsystems=sorted(remaining_stopped),
    )
    atomic_write_json(runtime_path(STATE_PATH), state)
    _save_event(
        config,
        {
            "event": "protection_recovery_failed",
            "timestamp": timestamp,
            "operator": operator,
            "onsite_confirmed": onsite_confirmed,
            "reason": reason,
            "state": state,
            "actions": actions,
        },
    )


def protect(
    level: str,
    *,
    reason: str,
    operator: str,
    scopes: list[str] | None = None,
    source: str = "manual_cli",
    dry_run: bool = False,
) -> dict[str, Any]:
    requested = level.upper()
    if requested not in ("P0", "P1", "P2"):
        raise ValueError("保护等级必须是 p0、p1 或 p2")
    scopes = sorted(set(scopes or []))
    unknown = sorted(set(scopes) - set(VALID_SCOPES))
    if unknown:
        raise ValueError(f"未知隔离范围：{', '.join(unknown)}")
    if requested == "P1" and not scopes:
        raise ValueError("P1 必须通过 --scope 指定至少一个子系统")
    if requested != "P1" and scopes:
        raise ValueError("只有 P1 接受 --scope")

    config = load_yaml("protection.yaml")
    timeout = float(config.get("command_timeout_seconds", 15))
    lock_context = (
        nullcontext()
        if dry_run
        else exclusive_lock(runtime_path("protection.lock"))
    )
    with lock_context:
        previous = load_state()
        previous_level = str(previous.get("state", "NORMAL")).upper()
        if LEVEL_RANK.get(previous_level, 0) > LEVEL_RANK[requested]:
            raise RuntimeError(
                f"当前处于 {previous_level}，不能直接降级到 {requested}；请先执行恢复"
            )
        previous_scopes = set(previous.get("isolated_subsystems", []))
        same_request = previous_level == requested and set(scopes).issubset(previous_scopes)
        if same_request:
            state = copy.deepcopy(previous)
            state.update(
                last_request_at=now_iso(),
                last_request_operator=operator,
                last_request_reason=reason,
            )
            atomic_write_json(runtime_path(STATE_PATH), state)
            return {"idempotent": True, "state": state, "actions": []}

        action_results: list[dict[str, Any]] = []
        executed_action_names: set[str] = set()
        action_order = (
            list(config.get("p0_actions", []))
            + list(config.get("common_cancel_actions", []))
            if requested == "P0"
            else list(config.get("common_cancel_actions", []))
        )
        # P0 中硬件急停/置零动作按配置放在取消任务之前，且同名动作只执行一次。
        for action_name in action_order:
            action_name = str(action_name)
            if action_name in executed_action_names:
                continue
            executed_action_names.add(action_name)
            action_results.append(
                run_action(
                    action_name,
                    _action_from_config(config, action_name),
                    timeout=timeout,
                    dry_run=dry_run,
                )
            )

        requested_scopes: list[str]
        if requested == "P1":
            requested_scopes = scopes
        elif requested == "P0":
            requested_scopes = sorted(
                key
                for key in config.get("managed_subsystems", {})
                if key in VALID_SCOPES
            )
        else:
            requested_scopes = []

        successfully_stopped = set(previous.get("stopped_subsystems", []))
        if requested in ("P1", "P0"):
            managed = config.get("managed_subsystems", {})
            for scope in requested_scopes:
                if scope in successfully_stopped:
                    continue
                subsystem = managed.get(scope, {}) if isinstance(managed, dict) else {}
                result = run_action(
                    f"{scope}.stop_action",
                    subsystem.get("stop_action", {}),
                    timeout=timeout,
                    dry_run=dry_run,
                )
                action_results.append(result)
                if result["success"] and not dry_run:
                    successfully_stopped.add(scope)

        disabled = [
            result["action"]
            for result in action_results
            if result["status"] in ("disabled", "invalid")
        ]
        failed = [
            result["action"]
            for result in action_results
            if result["status"] in ("failed", "timeout", "error")
        ]
        hardware_confirmed = any(
            result["action"] == "hardware_estop"
            and result["success"]
            and not dry_run
            for result in action_results
        )
        complete = not disabled and not failed
        timestamp = now_iso()
        state = {
            "schema_version": 1,
            "state": requested,
            "active": True,
            "event_count": int(previous.get("event_count", 0)) + 1,
            "reason": reason,
            "operator": operator,
            "source": source,
            "activated_at": timestamp,
            "updated_at": timestamp,
            "last_request_at": timestamp,
            "last_request_operator": operator,
            "isolated_subsystems": sorted(previous_scopes | set(requested_scopes)),
            "stopped_subsystems": sorted(successfully_stopped),
            "protection_complete": complete,
            "hardware_estop_confirmed": hardware_confirmed
            or bool(previous.get("hardware_estop_confirmed", False)),
            "missing_actions": disabled,
            "failed_actions": failed,
            "successful_actions": sorted(
                {
                    str(result["action"])
                    for result in action_results
                    if result.get("success") and not dry_run
                }
                | set(previous.get("successful_actions", []))
            ),
            "dry_run": dry_run,
        }
        event = {
            "event": "protection_activated",
            "timestamp": timestamp,
            "previous_state": previous_level,
            "state": state,
            "actions": action_results,
        }
        if dry_run:
            return {
                "idempotent": False,
                "dry_run": True,
                "state": state,
                "actions": action_results,
                "note": "dry-run 未写状态、事件日志或 ROS 告警。",
            }
        atomic_write_json(runtime_path(STATE_PATH), state)
        _save_event(config, event)

    qualifier = "" if complete else "；部分接口未配置或执行失败，保护效果未完全确认"
    alarm = make_alarm(
        "manual_protection",
        active=True,
        severity={"P2": "warning", "P1": "critical", "P0": "emergency"}[requested],
        source="protection",
        item=f"人工触发 {requested} 保护：{reason}{qualifier}",
        protection_state=requested,
        advice="保持机器人待命并按恢复流程逐项检查",
        onsite_required=requested == "P0",
        details={"missing_actions": disabled, "failed_actions": failed},
    )
    publish_error = _publish_alarm_best_effort(alarm)
    return {
        "idempotent": False,
        "state": state,
        "actions": action_results,
        "alarm_publish_error": publish_error,
    }


def recover(
    *,
    operator: str,
    onsite_confirmed: bool,
    dry_run: bool = False,
) -> dict[str, Any]:
    config = load_yaml("protection.yaml")
    timeout = float(config.get("command_timeout_seconds", 15))
    lock_context = (
        nullcontext()
        if dry_run
        else exclusive_lock(runtime_path("protection.lock"))
    )
    with lock_context:
        previous = load_state()
        previous_level = str(previous.get("state", "NORMAL")).upper()
        if previous_level == "NORMAL":
            return {"idempotent": True, "state": previous, "actions": []}
        if previous_level == "P0" and not onsite_confirmed:
            raise RuntimeError("P0 恢复必须提供 --onsite-confirmed")

        actions: list[dict[str, Any]] = []
        remaining_stopped = set(previous.get("stopped_subsystems", []))
        estop_action = _action_from_config(config, "physical_estop_released")
        if estop_action.get("enabled"):
            estop_check = run_action(
                "physical_estop_released",
                estop_action,
                timeout=timeout,
                dry_run=dry_run,
            )
            actions.append(estop_check)
            if not estop_check["success"]:
                _record_recovery_failure(
                    config,
                    previous,
                    operator=operator,
                    onsite_confirmed=onsite_confirmed,
                    actions=actions,
                    reason="物理急停未确认释放",
                    remaining_stopped=remaining_stopped,
                    dry_run=dry_run,
                )
                raise RuntimeError("物理急停未确认释放，拒绝恢复")
        elif not (
            onsite_confirmed
            and bool(
                config.get("recovery", {}).get("allow_onsite_estop_fallback", False)
            )
        ):
            reason = "尚未配置物理急停读取接口；必须现场确认后使用 --onsite-confirmed"
            _record_recovery_failure(
                config,
                previous,
                operator=operator,
                onsite_confirmed=onsite_confirmed,
                actions=actions,
                reason=reason,
                remaining_stopped=remaining_stopped,
                dry_run=dry_run,
            )
            raise RuntimeError(reason)

        clear_action = _action_from_config(config, "clear_software_protection")
        unlock_action = _action_from_config(config, "unlock_tasks")
        managed = config.get("managed_subsystems", {})
        start_actions: list[tuple[str, dict[str, Any]]] = []
        for scope in reversed(previous.get("stopped_subsystems", [])):
            subsystem = managed.get(scope, {}) if isinstance(managed, dict) else {}
            start_actions.append(
                (
                    scope,
                    _validate_action(
                        subsystem.get("start_action", {}), f"{scope}.start_action"
                    ),
                )
            )

        required_actions: list[tuple[str, dict[str, Any]]] = []
        if previous.get("hardware_estop_confirmed"):
            required_actions.append(("clear_software_protection", clear_action))
        required_actions.extend(
            (f"{scope}.start_action", action) for scope, action in start_actions
        )
        if "lock_tasks" in previous.get("successful_actions", []):
            required_actions.append(("unlock_tasks", unlock_action))
        unavailable = [
            label
            for label, action in required_actions
            if not action.get("enabled") or not action.get("argv")
        ]
        if unavailable:
            reason = f"恢复白名单动作未配置：{', '.join(unavailable)}"
            _record_recovery_failure(
                config,
                previous,
                operator=operator,
                onsite_confirmed=onsite_confirmed,
                actions=actions,
                reason=reason,
                remaining_stopped=remaining_stopped,
                dry_run=dry_run,
            )
            raise RuntimeError(reason)

        if previous.get("hardware_estop_confirmed"):
            clear_result = run_action(
                "clear_software_protection",
                clear_action,
                timeout=timeout,
                dry_run=dry_run,
            )
            actions.append(clear_result)
            if not clear_result["success"]:
                _record_recovery_failure(
                    config,
                    previous,
                    operator=operator,
                    onsite_confirmed=onsite_confirmed,
                    actions=actions,
                    reason="硬件/软件急停清除动作未成功",
                    remaining_stopped=remaining_stopped,
                    dry_run=dry_run,
                )
                raise RuntimeError("硬件/软件急停清除动作未成功，拒绝恢复")

        for scope, start_action in start_actions:
            result = run_action(
                f"{scope}.start_action",
                start_action,
                timeout=timeout,
                dry_run=dry_run,
            )
            actions.append(result)
            if not result["success"]:
                _record_recovery_failure(
                    config,
                    previous,
                    operator=operator,
                    onsite_confirmed=onsite_confirmed,
                    actions=actions,
                    reason=f"{scope} 未能按白名单恢复",
                    remaining_stopped=remaining_stopped,
                    dry_run=dry_run,
                )
                raise RuntimeError(f"{scope} 未能按白名单恢复，保护状态保持不变")
            if not dry_run:
                remaining_stopped.discard(scope)

        unlock = run_action(
            "unlock_tasks",
            unlock_action,
            timeout=timeout,
            dry_run=dry_run,
        )
        actions.append(unlock)
        if (
            "lock_tasks" in previous.get("successful_actions", [])
            and not unlock["success"]
        ):
            _record_recovery_failure(
                config,
                previous,
                operator=operator,
                onsite_confirmed=onsite_confirmed,
                actions=actions,
                reason="任务入口解锁动作未成功",
                remaining_stopped=remaining_stopped,
                dry_run=dry_run,
            )
            raise RuntimeError("任务入口曾成功锁定，但解锁动作未成功，保护状态保持不变")
        zero = run_action(
            "zero_base",
            _action_from_config(config, "zero_base"),
            timeout=timeout,
            dry_run=dry_run,
        )
        actions.append(zero)

        timestamp = now_iso()
        state = default_state()
        state.update(
            event_count=int(previous.get("event_count", 0)),
            updated_at=timestamp,
            recovered_at=timestamp,
            recovered_by=operator,
            recovered_from=previous_level,
            onsite_confirmed=onsite_confirmed,
            # 恢复只回到待命，不保存或重放旧任务。
            old_task_replayed=False,
        )
        if dry_run:
            return {
                "idempotent": False,
                "dry_run": True,
                "state": state,
                "actions": actions,
                "note": "dry-run 未写状态、事件日志或 ROS 告警。",
            }
        atomic_write_json(runtime_path(STATE_PATH), state)
        _save_event(
            config,
            {
                "event": "protection_recovered",
                "timestamp": timestamp,
                "operator": operator,
                "previous_state": previous,
                "state": state,
                "actions": actions,
            },
        )

    alarm = make_alarm(
        "manual_protection",
        active=False,
        source="protection",
        item=f"{previous_level} 保护已由 {operator} 解除，机器人保持待命",
        protection_state="NORMAL",
        advice="确认任务队列为空后再人工下发新任务",
    )
    publish_error = _publish_alarm_best_effort(alarm)
    return {
        "idempotent": False,
        "state": state,
        "actions": actions,
        "alarm_publish_error": publish_error,
    }


def status_text(state: dict[str, Any]) -> str:
    if state.get("state") == "NORMAL":
        return "\n".join(
            (
                "【MOS 运维保护状态】",
                "状态：NORMAL（待命）",
                f"保护事件计数：{state.get('event_count', 0)}",
                f"更新时间：{state.get('updated_at', '未知')}",
            )
        )
    complete = "是" if state.get("protection_complete") else "否"
    dry_run_notice = "（DRY-RUN 预览，未实际生效）" if state.get("dry_run") else ""
    return "\n".join(
        (
            "【MOS 运维保护状态】",
            f"状态：{state.get('state')}{dry_run_notice}",
            f"原因：{state.get('reason', '未填写')}",
            f"操作人：{state.get('operator', '未知')}",
            f"隔离范围：{', '.join(state.get('isolated_subsystems', [])) or '无'}",
            f"已确认停止：{', '.join(state.get('stopped_subsystems', [])) or '无'}",
            f"保护动作完整成功：{complete}",
            f"缺少动作：{', '.join(state.get('missing_actions', [])) or '无'}",
            f"失败动作：{', '.join(state.get('failed_actions', [])) or '无'}",
            f"事件计数：{state.get('event_count', 0)}",
            f"更新时间：{state.get('updated_at', '未知')}",
        )
    )


def _print_result(result: dict[str, Any], as_json: bool) -> None:
    if as_json:
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(status_text(result["state"]))
        if result.get("idempotent"):
            print("说明：同一保护请求已幂等处理，未重复执行动作或增加计数。")
        elif result.get("alarm_publish_error"):
            print(f"提示：本地状态已保存，但 ROS 告警发布失败：{result['alarm_publish_error']}")


def protect_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="人工触发 MOS 软件保护")
    parser.add_argument("level", choices=("p0", "p1", "p2"))
    parser.add_argument("--reason", required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--scope", action="append", choices=VALID_SCOPES, default=[])
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = protect(
            args.level,
            reason=args.reason,
            operator=args.operator,
            scopes=args.scope,
            dry_run=args.dry_run,
        )
        _print_result(result, args.json)
        return 0
    except (RuntimeError, ValueError) as exc:
        print(f"保护请求失败：{exc}", file=sys.stderr)
        return 2


def recover_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="人工恢复 MOS 到待命状态")
    parser.add_argument("--operator", required=True)
    parser.add_argument("--onsite-confirmed", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    try:
        result = recover(
            operator=args.operator,
            onsite_confirmed=args.onsite_confirmed,
            dry_run=args.dry_run,
        )
        _print_result(result, args.json)
        return 0
    except (RuntimeError, ValueError) as exc:
        print(f"恢复请求失败：{exc}", file=sys.stderr)
        return 2


def status_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="查看 MOS 运维保护状态")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    state = load_state()
    print(
        json.dumps(state, ensure_ascii=False, indent=2, sort_keys=True)
        if args.json
        else status_text(state)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(protect_main())
