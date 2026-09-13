#!/usr/bin/env python3
"""导航点与符号表加载。

命名约定:
  A      <-> patrol_point_a   (巡航点)
  ACart  <-> approach_point_a (接近点)

主模块用法::

    from ... import load_nav  # 见 coordinator_node 中的导入方式
    result = load_nav.load_navigation_symbols(robot_id='R1', ros_node=self)
    self.navigation_yaml = result.navigation_yaml
    self.SYMBOL_LST = result.SYMBOL_LST
    self.CartSymbolLst = result.CartSymbolLst
    self.CartSymbols = result.CartSymbols
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, NamedTuple, Optional, Tuple


# 各机器人各阶段默认巡航符号
DEFAULT_PHASE_ROUTES: Dict[str, Dict[str, List[str]]] = {
    'R1': {
        'SearchFull': ['patrol_point_a', 'patrol_point_b'],
        'SearchEmpty': ['patrol_point_d', 'patrol_point_e'],
        'PutFull': ['patrol_point_d', 'patrol_point_e'],
        'PutEmpty': ['patrol_point_a', 'patrol_point_b'],
    },
    'R2': {
        'SearchFull': ['patrol_point_d', 'patrol_point_e'],
        'SearchEmpty': ['patrol_point_f', 'patrol_point_g'],
        'PutFull': ['patrol_point_f', 'patrol_point_g'],
        'PutEmpty': ['patrol_point_d', 'patrol_point_e'],
    },
}

DEFAULT_CART_SYMBOL_LST: List[str] = [
    'approach_point_a',
    'approach_point_b',
    'approach_point_c',
    'approach_point_d',
    'approach_point_e',
    'approach_point_f',
    'approach_point_g',
]

DEFAULT_CART_SYMBOLS: Dict[str, str] = {
    'patrol_point_a': 'approach_point_a',
    'patrol_point_b': 'approach_point_b',
    'patrol_point_c': 'approach_point_c',
    'patrol_point_d': 'approach_point_d',
    'patrol_point_e': 'approach_point_e',
    'patrol_point_f': 'approach_point_f',
    'patrol_point_g': 'approach_point_g',
}

# 默认坐标（yaml / 参数未覆盖时使用）
DEFAULT_POINT_COORDS: Dict[str, Tuple[float, float, float]] = {
    'a': (1.0, 1.0, 0.0),
    'b': (2.0, 1.0, 0.0),
    'd': (0.3, 0.3, 0.0),
    'e': (-1.2, 0.3, 0.0),
    'f': (5.0, 1.0, 0.0),
    'g': (6.0, 1.0, 0.0),
}

DEFAULT_APPROACH_COORDS: Dict[str, Tuple[float, float, float]] = {
    'a': (1.0, 1.5, 0.0),
    'b': (2.0, 1.5, 0.0),
    'd': (0.3, 0.8, 0.0),
    'e': (-1.2, 0.8, 0.0),
    'f': (5.0, 1.5, 0.0),
    'g': (6.0, 1.5, 0.0),
}

PointTuple = Tuple[float, float, float, str]


class NavLoadResult(NamedTuple):
    """load_navigation_symbols 的返回值。"""

    navigation_yaml: Dict[str, Dict[str, PointTuple]]
    SYMBOL_LST: Dict[str, Dict[str, List[str]]]
    CartSymbolLst: List[str]
    CartSymbols: Dict[str, str]
    config_path: Optional[str] = None


class _PrintLogger:
    def info(self, msg: str) -> None:
        print(f'[INFO] {msg}')

    def warn(self, msg: str) -> None:
        print(f'[WARN] {msg}')

    def warning(self, msg: str) -> None:
        print(f'[WARN] {msg}')

    def error(self, msg: str) -> None:
        print(f'[ERROR] {msg}')


def resolve_coordinator_params_path(config_path: Optional[str] = None) -> str:
    """解析 coordinator_params.yaml 路径。"""
    if config_path:
        return os.path.abspath(config_path)

    try:
        from ament_index_python.packages import get_package_share_directory
        installed = os.path.join(
            get_package_share_directory('mos_coordinator'),
            'config',
            'coordinator_params.yaml',
        )
        if os.path.isfile(installed):
            return installed
    except Exception:
        pass

    # config/load_nav.py 同目录下的 yaml
    return os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'coordinator_params.yaml')
    )


def point_dict_to_tuple(point: dict, frame_id: str = 'map') -> PointTuple:
    """将点位字典转为 (x, y, yaw, frame)。"""
    return (
        float(point.get('x', 0.0)),
        float(point.get('y', 0.0)),
        float(point.get('yaw', 0.0)),
        frame_id,
    )


def load_nav_points_from_yaml_file(
    config_path: Optional[str] = None,
    logger: Any = None,
) -> Optional[Dict[str, PointTuple]]:
    """解析 coordinator_params.yaml 中的巡航/接近点。

    Returns:
        符号 -> (x, y, yaw, frame)；失败时返回 None
    """
    log = logger or _PrintLogger()
    try:
        import yaml
    except ImportError:
        log.warn('未安装 PyYAML，改用 ROS 参数加载导航点')
        return None

    path = resolve_coordinator_params_path(config_path)
    if not os.path.isfile(path):
        log.warn(f'未找到配置文件: {path}，改用 ROS 参数加载导航点')
        return None

    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}

    params = raw
    if isinstance(raw, dict) and '/**' in raw:
        params = raw['/**'].get('ros__parameters', {}) or {}
    elif isinstance(raw, dict) and 'ros__parameters' in raw:
        params = raw.get('ros__parameters', {}) or {}

    nav_points: Dict[str, PointTuple] = {}
    for key, value in params.items():
        if not isinstance(value, dict):
            continue
        nav_points[key] = point_dict_to_tuple(value)

    log.info(f'已从配置文件加载导航点: {path}')
    return nav_points


def load_nav_points_from_ros_params(
    ros_node: Any,
    logger: Any = None,
) -> Dict[str, PointTuple]:
    """从 ROS 参数服务器加载巡航/接近点（launch 注入 yaml 时生效）。"""
    log = logger or _PrintLogger()
    letters = sorted(set(DEFAULT_POINT_COORDS) | set(DEFAULT_APPROACH_COORDS))

    nav_points: Dict[str, PointTuple] = {}
    for letter in letters:
        upper = letter.upper()
        patrol_prefix = f'patrol_point_{letter}'
        approach_prefix = f'approach_point_{letter}'

        px, py, pyaw = DEFAULT_POINT_COORDS.get(letter, (0.0, 0.0, 0.0))
        ros_node.declare_parameter(f'{patrol_prefix}.x', float(px))
        ros_node.declare_parameter(f'{patrol_prefix}.y', float(py))
        ros_node.declare_parameter(f'{patrol_prefix}.z', 0.0)
        ros_node.declare_parameter(f'{patrol_prefix}.yaw', float(pyaw))
        nav_points[upper] = (
            float(ros_node.get_parameter(f'{patrol_prefix}.x').value),
            float(ros_node.get_parameter(f'{patrol_prefix}.y').value),
            float(ros_node.get_parameter(f'{patrol_prefix}.yaw').value),
            'map',
        )

        ax, ay, ayaw = DEFAULT_APPROACH_COORDS.get(letter, (0.0, 0.0, 0.0))
        ros_node.declare_parameter(f'{approach_prefix}.x', float(ax))
        ros_node.declare_parameter(f'{approach_prefix}.y', float(ay))
        ros_node.declare_parameter(f'{approach_prefix}.z', 0.0)
        ros_node.declare_parameter(f'{approach_prefix}.yaw', float(ayaw))
        nav_points[f'{upper}Cart'] = (
            float(ros_node.get_parameter(f'{approach_prefix}.x').value),
            float(ros_node.get_parameter(f'{approach_prefix}.y').value),
            float(ros_node.get_parameter(f'{approach_prefix}.yaw').value),
            'map',
        )

    log.info('已从 ROS 参数加载导航点')
    return nav_points


def assign_symbols_from_navigation(
    navigation_yaml: Dict[str, Dict[str, PointTuple]],
) -> Tuple[Dict[str, Dict[str, List[str]]], List[str], Dict[str, str]]:
    """根据 navigation_yaml 分配 SYMBOL_LST / CartSymbolLst / CartSymbols。

    当前实现沿用默认阶段路由与接近点映射表；
    navigation_yaml 中的 A/ACart 坐标供 get_pose_by_symbol 使用。
    """
    #_ = navigation_yaml.get(robot_id, {})
    symbol_lst = {
        rid: {phase: list(symbols) for phase, symbols in routes.items()}
        for rid, routes in DEFAULT_PHASE_ROUTES.items()
    }
    cart_symbol_lst = list(DEFAULT_CART_SYMBOL_LST)
    cart_symbols = dict(DEFAULT_CART_SYMBOLS)
    return symbol_lst, cart_symbol_lst, cart_symbols


def load_navigation_symbols(
    robot_id: str = 'R1',
    config_path: Optional[str] = None,
    ros_node: Any = None,
    logger: Any = None,
) -> NavLoadResult:
    """从 coordinator_params.yaml 读点，填充 navigation_yaml 并分配符号表。

    命名约定:
      A      -> patrol_point_a
      ACart  -> approach_point_a
    """
    log = logger or _PrintLogger()
    path = resolve_coordinator_params_path(config_path)

    nav_points = load_nav_points_from_yaml_file(config_path=path, logger=log)
    log.info(f'nav_points: {nav_points}')
    if not nav_points:
        if ros_node is None:
            log.error('yaml 加载失败且未提供 ros_node，返回空结果')
            return NavLoadResult(
                navigation_yaml={},
                SYMBOL_LST={},
                CartSymbolLst=[],
                CartSymbols={},
                config_path=path,
            )
        nav_points = load_nav_points_from_ros_params(ros_node=ros_node, logger=log)

    log.info(f'nav_points: {nav_points}')

    navigation_yaml = {
        rid: dict(nav_points) for rid in DEFAULT_PHASE_ROUTES
    }
    log.info(f'navigation_yaml: {navigation_yaml}')
    symbol_lst, cart_lst, cart_map = assign_symbols_from_navigation(
        navigation_yaml
    )

    log.info(f'navigation_yaml: {navigation_yaml}')
    result = NavLoadResult(
        navigation_yaml=navigation_yaml,
        SYMBOL_LST=symbol_lst,
        CartSymbolLst=cart_lst,
        CartSymbols=cart_map,
        config_path=path,
    )

    log.info(f'导航点符号: {sorted(nav_points.keys())}')
    log.info(f'符号表 SYMBOL_LST[{robot_id}]={result.SYMBOL_LST.get(robot_id)}')
    log.info(f'CartSymbols={result.CartSymbols}')
    return result


def apply_to_node(node: Any, config_path: Optional[str] = None) -> NavLoadResult:
    """将加载结果写到 node 的常用属性上（供 TaskCoordinator 调用）。"""
    robot_id = getattr(node, 'robot_id', 'R1')
    logger = node.get_logger() if hasattr(node, 'get_logger') else None
    result = load_navigation_symbols(
        robot_id=robot_id,
        config_path=config_path,
        ros_node=node,
        logger=logger,
    )
    node.navigation_yaml = result.navigation_yaml
    node.SYMBOL_LST = result.SYMBOL_LST
    node.CartSymbolLst = result.CartSymbolLst
    node.CartSymbols = result.CartSymbols
    return result
