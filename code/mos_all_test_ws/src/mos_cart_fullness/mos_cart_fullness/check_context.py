import json
import math
from dataclasses import dataclass


VALID_MODES = frozenset({"scan_wrist", "scan_dual", "verify_dual"})


@dataclass(frozen=True)
class CheckContext:
    check_id: str
    mode: str
    waypoint: str
    zone_id: str
    cart_id: str
    wrist_camera: str
    timeout_sec: float
    legacy: bool = False

    @property
    def uses_head(self) -> bool:
        return self.mode in {"scan_dual", "verify_dual"}


@dataclass(frozen=True)
class ParsedCommand:
    action: str
    context: CheckContext | None = None


def parse_check_command(
    data: str,
    *,
    waypoint_camera_map: dict[str, str],
    waypoint_cart_map: dict[str, str],
    default_verify_timeout_sec: float,
) -> ParsedCommand:
    text = str(data or "").strip()
    if text in {"", "off", "disable", "disabled"}:
        return ParsedCommand(action="off")
    if text == "reset":
        return ParsedCommand(action="reset")

    try:
        payload = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        payload = None

    if not isinstance(payload, dict):
        waypoint = text
        camera = waypoint_camera_map.get(waypoint, "")
        if camera not in {"left", "right"}:
            raise ValueError(f"no camera mapping for waypoint {waypoint!r}")
        return ParsedCommand(
            action="start",
            context=CheckContext(
                check_id=f"legacy:{waypoint}",
                mode="scan_wrist",
                waypoint=waypoint,
                zone_id=waypoint,
                cart_id=waypoint_cart_map.get(waypoint, waypoint),
                wrist_camera=camera,
                timeout_sec=0.0,
                legacy=True,
            ),
        )

    waypoint = str(payload.get("waypoint") or payload.get("data") or "").strip()
    has_extended_fields = any(
        key in payload
        for key in ("check_id", "mode", "zone_id", "cart_id", "wrist_camera")
    )
    if not has_extended_fields:
        return parse_check_command(
            waypoint,
            waypoint_camera_map=waypoint_camera_map,
            waypoint_cart_map=waypoint_cart_map,
            default_verify_timeout_sec=default_verify_timeout_sec,
        )

    check_id = str(payload.get("check_id") or "").strip()
    mode = str(payload.get("mode") or "").strip()
    wrist_camera = str(payload.get("wrist_camera") or "").strip()
    if not check_id:
        raise ValueError("extended command requires check_id")
    if mode not in VALID_MODES:
        raise ValueError(f"unsupported mode: {mode!r}")
    if wrist_camera not in {"left", "right"}:
        raise ValueError(f"unsupported wrist_camera: {wrist_camera!r}")
    raw_timeout = payload.get("timeout_sec", 0.0)
    if raw_timeout is None or isinstance(raw_timeout, bool):
        raise ValueError("timeout_sec must be a finite number")
    try:
        timeout_sec = float(raw_timeout)
    except (TypeError, ValueError) as exc:
        raise ValueError("timeout_sec must be a finite number") from exc
    if not math.isfinite(timeout_sec):
        raise ValueError("timeout_sec must be a finite number")
    if mode == "verify_dual" and timeout_sec <= 0.0:
        timeout_sec = max(0.01, float(default_verify_timeout_sec))
    return ParsedCommand(
        action="start",
        context=CheckContext(
            check_id=check_id,
            mode=mode,
            waypoint=waypoint,
            zone_id=str(payload.get("zone_id") or waypoint).strip(),
            cart_id=str(payload.get("cart_id") or waypoint).strip(),
            wrist_camera=wrist_camera,
            timeout_sec=max(0.0, timeout_sec),
            legacy=False,
        ),
    )


def fuse_view_decisions(
    mode: str,
    wrist_decision: str,
    head_decision: str,
    timed_out: bool,
) -> str:
    if mode == "scan_wrist":
        return wrist_decision if wrist_decision in {"full", "not_full"} else ""
    # Each view is evaluated independently. A missing/unknown view must not
    # block a decisive result from the view that currently has an image.
    if wrist_decision == "not_full" or head_decision == "not_full":
        return "not_full"
    if wrist_decision == "full" or head_decision == "full":
        return "full"
    if mode == "verify_dual" and timed_out:
        return "not_full"
    return ""
