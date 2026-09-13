#!/usr/bin/env python3
"""独立测试 config/load_nav.py

命名约定:
  A      <-> patrol_point_a   (巡航点)
  ACart  <-> approach_point_a (接近点)

用法:
  python3 test/scripts/test_load_navigation_symbols.py
  python3 test/scripts/test_load_navigation_symbols.py --config /path/to/coordinator_params.yaml
  python3 test/scripts/test_load_navigation_symbols.py --robot-id R2
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from pprint import pprint


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.normpath(os.path.join(_SCRIPT_DIR, '..', '..'))
_DEFAULT_CONFIG = os.path.join(_PKG_ROOT, 'config', 'coordinator_params.yaml')
_LOAD_NAV_PATH = os.path.join(_PKG_ROOT, 'config', 'load_nav.py')


def _import_load_nav():
    mod_name = 'load_nav'
    if mod_name in sys.modules:
        return sys.modules[mod_name]
    spec = importlib.util.spec_from_file_location(mod_name, _LOAD_NAV_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = module
    spec.loader.exec_module(module)
    return module


def _print_section(title: str):
    print()
    print('=' * 60)
    print(title)
    print('=' * 60)


def main():
    parser = argparse.ArgumentParser(description='测试 config/load_nav.load_navigation_symbols')
    parser.add_argument(
        '--config',
        default=_DEFAULT_CONFIG,
        help=f'coordinator_params.yaml 路径 (默认: {_DEFAULT_CONFIG})',
    )
    parser.add_argument('--robot-id', default='R1', choices=('R1', 'R2'))
    args = parser.parse_args()

    config_path = os.path.abspath(args.config)
    if not os.path.isfile(config_path):
        print(f'[ERROR] 配置文件不存在: {config_path}')
        return 1
    if not os.path.isfile(_LOAD_NAV_PATH):
        print(f'[ERROR] 未找到 load_nav.py: {_LOAD_NAV_PATH}')
        return 1

    try:
        import yaml  # noqa: F401
    except ImportError:
        print('[ERROR] 需要 PyYAML。请执行: pip install pyyaml')
        return 1

    load_nav = _import_load_nav()

    _print_section(f'调用 load_navigation_symbols  (robot_id={args.robot_id})')
    print(f'config : {config_path}')
    print(f'load_nav: {_LOAD_NAV_PATH}')
    result = load_nav.load_navigation_symbols(
        robot_id=args.robot_id,
        config_path=config_path,
    )

    nav = result.navigation_yaml.get(args.robot_id, {})

    _print_section('SYMBOL_LST')
    pprint(result.SYMBOL_LST, width=100, sort_dicts=False)

    _print_section('CartSymbolLst')
    pprint(result.CartSymbolLst, width=100)

    _print_section('CartSymbols  (巡航点 -> 接近点)')
    pprint(result.CartSymbols, width=100, sort_dicts=False)

    _print_section(
        'navigation_yaml 命名约定核对 '
        '(A->patrol_point_x, ACart->approach_point_x)'
    )
    patrol_syms = sorted(k for k in nav if not str(k).endswith('Cart'))
    cart_syms = sorted(k for k in nav if str(k).endswith('Cart'))
    print(f'巡航点符号: {patrol_syms}')
    print(f'接近点符号: {cart_syms}')
    print()

    letters = set()
    for s in patrol_syms:
        if len(str(s)) == 1:
            letters.add(str(s).upper())
    for s in cart_syms:
        base = str(s)[:-4] if str(s).endswith('Cart') else str(s)
        if len(base) == 1:
            letters.add(base.upper())

    for letter in sorted(letters):
        patrol_key = f'patrol_point_{letter.lower()}'
        approach_key = f'approach_point_{letter.lower()}'
        print(
            f'  {letter:6} <- {patrol_key:18}  '
            f"{nav.get(letter, '(yaml中无)')}"
        )
        print(
            f'  {letter + "Cart":6} <- {approach_key:18}  '
            f"{nav.get(letter + 'Cart', '(yaml中无)')}"
        )

    _print_section('结论摘要')
    print(f'robot_id       : {args.robot_id}')
    print(f'SYMBOL_LST[{args.robot_id}]: {result.SYMBOL_LST.get(args.robot_id)}')
    print(f'CartSymbolLst  : {result.CartSymbolLst}')
    print(f'CartSymbols    : {result.CartSymbols}')
    print(f'result.navigation_yaml : {result.navigation_yaml}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
