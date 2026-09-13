from __future__ import annotations

import json
from typing import Any, Iterable

from .storage import now_iso


SEVERITY_ORDER = {"warning": 1, "critical": 2, "emergency": 3}
SEVERITY_CN = {"warning": "警告", "critical": "严重", "emergency": "紧急"}


def make_alarm(
    alarm_id: str,
    *,
    active: bool,
    severity: str = "warning",
    source: str,
    item: str,
    protection_state: str = "NORMAL",
    advice: str = "请检查设备状态",
    onsite_required: bool = False,
    details: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if severity not in SEVERITY_ORDER:
        raise ValueError(f"未知告警等级：{severity}")
    return {
        "schema_version": 1,
        "alarm_id": alarm_id,
        "active": bool(active),
        "severity": severity,
        "source": source,
        "item": item,
        "protection_state": protection_state,
        "advice": advice,
        "onsite_required": bool(onsite_required),
        "timestamp": now_iso(),
        "details": details or {},
    }


def alarm_to_json(alarm: dict[str, Any]) -> str:
    return json.dumps(alarm, ensure_ascii=False, sort_keys=True)


def parse_alarm(raw: str) -> dict[str, Any]:
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("告警消息必须是 JSON 对象")
    required = ("alarm_id", "active", "source")
    missing = [key for key in required if key not in value]
    if missing:
        raise ValueError(f"告警消息缺少字段：{', '.join(missing)}")
    severity = value.get("severity", "warning")
    if severity not in SEVERITY_ORDER:
        raise ValueError(f"未知告警等级：{severity}")
    return value


def highest_alarm(alarms: Iterable[dict[str, Any]]) -> dict[str, Any] | None:
    active = [alarm for alarm in alarms if alarm.get("active")]
    if not active:
        return None
    return max(
        active,
        key=lambda alarm: (
            SEVERITY_ORDER.get(str(alarm.get("severity")), 0),
            str(alarm.get("timestamp", "")),
        ),
    )


def format_alarm(alarm: dict[str, Any], active_count: int = 1) -> str:
    level = SEVERITY_CN.get(str(alarm.get("severity")), str(alarm.get("severity")))
    extra = f"（当前共 {active_count} 项）" if active_count > 1 else ""
    return "\n".join(
        (
            "【MOS异常】",
            f"等级：{level}{extra}",
            f"时间：{alarm.get('timestamp', '未知')}",
            f"来源：{alarm.get('source', '未知')}",
            f"异常项：{alarm.get('item', '未知')}",
            f"当前保护状态：{alarm.get('protection_state', 'NORMAL')}",
            f"建议动作：{alarm.get('advice', '请检查设备状态')}",
            f"是否需要现场人员：{'是' if alarm.get('onsite_required') else '否'}",
        )
    )
