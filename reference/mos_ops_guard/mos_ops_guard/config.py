from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def project_root() -> Path:
    override = os.environ.get("MOS_OPS_GUARD_ROOT")
    return Path(override).expanduser().resolve() if override else PROJECT_ROOT


def config_path(name: str) -> Path:
    return project_root() / "config" / name


def runtime_path(name: str) -> Path:
    path = project_root() / "runtime" / name
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def load_yaml(name: str) -> dict[str, Any]:
    path = config_path(name)
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"配置文件不存在：{path}") from exc
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise RuntimeError(f"无法读取配置文件 {path}：{exc}") from exc
    if not isinstance(value, dict):
        raise RuntimeError(f"配置文件顶层必须是映射：{path}")
    return value


def expand_path(raw: str) -> Path:
    return Path(os.path.expandvars(raw)).expanduser()

