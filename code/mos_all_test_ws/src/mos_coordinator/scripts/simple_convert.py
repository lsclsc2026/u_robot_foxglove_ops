#!/usr/bin/env python3
"""
简单的坐标转换脚本：target_goals.yaml → coordinator_params.yaml

功能：
  将真机上的 /tmp/mos_all_test_ws/nav2_waypoints.yaml 转换为
  coordinator_params.yaml 格式

输入格式：
  a:
    name: a
    position: {x: 1.6916, y: 0.2949, z: -0.0747}
    orientation: {x: -0.0051, y: -0.001, z: -0.0252, w: 0.9997}

输出格式：
  observation_point_a:
    x: 1.6916
    y: 0.2949
    z: -0.0747
    yaw: -0.0504

用法：
  python3 simple_convert.py input.yaml output.yaml
"""

import yaml
import math
import sys
from pathlib import Path


def quaternion_to_yaw(qx, qy, qz, qw):
    """四元数转 yaw 角度（弧度）

    使用简化公式：yaw = 2 * atan2(qz, qw)
    """
    return 2 * math.atan2(qz, qw)


def convert_points(input_data):
    """转换所有点位 - 支持任意名称

    命名规则：
    - 单字母/名称（如 a, d, point1）-> observation_point_*
    - 双字母/名称+后缀（如 aa, dd, point1a）-> approach_point_*
    """
    observation_points = []
    approach_points = []

    for key, data in input_data.items():
        # 验证数据结构
        if not isinstance(data, dict):
            continue
        if 'position' not in data or 'orientation' not in data:
            continue

        point_name = key.lower()
        pos = data['position']
        ori = data['orientation']
        yaw = quaternion_to_yaw(ori['x'], ori['y'], ori['z'], ori['w'])

        point_data = {
            'original_name': key,
            'name': point_name,
            'x': pos['x'],
            'y': pos['y'],
            'z': pos['z'],
            'yaw': round(yaw, 4),
            'quat': f"x={ori['x']}, y={ori['y']}, z={ori['z']}, w={ori['w']}"
        }

        # 判断是观察点还是接近点
        # 规则：如果名称是两个相同字母（如 aa, bb, dd），或者以 a 结尾且长度>1，则是接近点
        if len(point_name) >= 2 and point_name[-1] == point_name[-2]:
            # 双字母情况：aa -> a, bb -> b
            base_name = point_name[:-1]
            point_data['base_name'] = base_name
            point_data['description'] = f"{key.upper()} - 接近点（对应 {base_name.upper()}）"
            approach_points.append(point_data)
        else:
            # 普通观察点
            point_data['base_name'] = point_name
            point_data['description'] = f"{key.upper()} - 观察点"
            observation_points.append(point_data)

    return observation_points, approach_points


def generate_yaml_content(observation_points, approach_points):
    """生成 YAML 内容

    Args:
        observation_points: 观察点列表
        approach_points: 接近点列表
    """
    lines = [
        "# MOS Coordinator 配置参数",
        "# 自动生成 - 从 target_goals.yaml 转换",
        "# 说明: 使用真实标定的坐标数据",
        "",
        "/**:",
        "  ros__parameters:",
        "    # === 机器人ID配置 ===",
        "    robot_id: \"R1\"  # 机器人标识，可选: \"R1\" 或 \"R2\"",
        "",
        "    # === 观察点配置（真实标定坐标） ===",
    ]

    # 添加每个观察点
    for p in observation_points:
        lines.extend([
            f"    # {p['description']}",
            f"    observation_point_{p['base_name']}:",
            f"      x: {p['x']:<10}  # {p['original_name']} X 坐标（米）",
            f"      y: {p['y']:<10}  # {p['original_name']} Y 坐标（米）",
            f"      z: {p['z']:<10}  # {p['original_name']} Z 坐标（米）",
            f"      # 四元数: {p['quat']}",
            f"      yaw: {p['yaw']:<9}  # {p['original_name']} 朝向角度（弧度）",
            "",
        ])

    # 添加巡航点配置（基于观察点）
    if observation_points:
        lines.extend([
            "    # === 巡航点配置（基于观察点，向后兼容） ===",
        ])

        for p in observation_points:
            lines.extend([
                f"    # 巡航点 {p['original_name'].upper()}",
                f"    patrol_point_{p['base_name']}:",
                f"      x: {p['x']}",
                f"      y: {p['y']}",
                f"      z: {p['z']}",
                f"      yaw: {p['yaw']}",
                "",
            ])

    # 添加接近点配置
    if approach_points:
        lines.extend([
            "    # === 接近点配置（抓取点，从 *a/*aa 等点位生成） ===",
        ])

        for p in approach_points:
            lines.extend([
                f"    # {p['description']}",
                f"    approach_point_{p['base_name']}:",
                f"      x: {p['x']:<10}  # {p['original_name']} X 坐标（米）",
                f"      y: {p['y']:<10}  # {p['original_name']} Y 坐标（米）",
                f"      z: {p['z']:<10}  # {p['original_name']} Z 坐标（米）",
                f"      yaw: {p['yaw']:<9}  # {p['original_name']} 朝向角度（弧度）",
                "",
            ])

    # 添加任务配置
    lines.extend([
        "    # === 任务配置 ===",
        "    # 状态发布频率（Hz）",
        "    status_publish_rate: 1.0",
        "",
        "    # 到达判断阈值（米）",
        "    arrival_threshold: 0.5",
        "",
        "    # 距离计算更新频率（Hz）",
        "    distance_check_rate: 2.0",
    ])

    return '\n'.join(lines)


def main():
    if len(sys.argv) < 3:
        print("用法: python3 simple_convert.py input.yaml output.yaml")
        print("")
        print("示例:")
        print("  python3 simple_convert.py /tmp/mos_all_test_ws/nav2_waypoints.yaml config/coordinator_params.yaml")
        sys.exit(1)

    input_file = Path(sys.argv[1])
    output_file = Path(sys.argv[2])

    print(f"📖 读取输入文件: {input_file}")

    if not input_file.exists():
        print(f"❌ 错误: 文件不存在: {input_file}")
        sys.exit(1)

    # 读取输入
    with open(input_file, 'r') as f:
        input_data = yaml.safe_load(f)

    print(f"✅ 成功读取 {len(input_data)} 个点位")

    # 转换
    print(f"🔄 转换坐标...")
    observation_points, approach_points = convert_points(input_data)

    if not observation_points and not approach_points:
        print(f"❌ 错误: 没有找到有效的点位")
        print(f"提示: 请确保输入文件包含 position 和 orientation 数据")
        sys.exit(1)

    total_points = len(observation_points) + len(approach_points)
    print(f"✅ 转换了 {total_points} 个点位:")
    if observation_points:
        obs_names = ', '.join([p['original_name'].upper() for p in observation_points])
        print(f"   - 观察点 ({len(observation_points)}个): {obs_names}")
    if approach_points:
        app_names = ', '.join([p['original_name'].upper() for p in approach_points])
        print(f"   - 接近点 ({len(approach_points)}个): {app_names}")

    # 生成输出
    print(f"📝 生成配置文件...")
    content = generate_yaml_content(observation_points, approach_points)

    # 写入输出
    output_file.parent.mkdir(parents=True, exist_ok=True)
    with open(output_file, 'w') as f:
        f.write(content)

    print(f"✅ 成功生成: {output_file}")
    print("")
    print("=" * 70)
    print("📍 转换后的坐标:")

    if observation_points:
        print("\n观察点 (observation_point_*):")
        print(f"{'点位':<10} {'X (米)':<12} {'Y (米)':<12} {'Z (米)':<12} {'Yaw (弧度)':<12}")
        print("-" * 70)
        for p in observation_points:
            print(f"{p['original_name']:<10} {p['x']:<12.4f} {p['y']:<12.4f} {p['z']:<12.4f} {p['yaw']:<12.4f}")

    if approach_points:
        print("\n接近点 (approach_point_*):")
        print(f"{'点位':<10} {'X (米)':<12} {'Y (米)':<12} {'Z (米)':<12} {'Yaw (弧度)':<12}")
        print("-" * 70)
        for p in approach_points:
            print(f"{p['original_name']:<10} {p['x']:<12.4f} {p['y']:<12.4f} {p['z']:<12.4f} {p['yaw']:<12.4f}")

    print("=" * 70)
    print("")
    print("✨ 完成！可以使用转换后的配置文件了。")


if __name__ == '__main__':
    main()
