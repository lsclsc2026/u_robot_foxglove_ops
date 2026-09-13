#!/usr/bin/env python3
"""Dummy Navigator - 极简导航模拟器

核心功能：模拟导航到指定位姿，立即返回成功。
适用于测试 coordinator 的状态机逻辑，无需真实导航环境。
"""

import math


class DummyNavigator:
    """极简导航模拟器"""

    def __init__(self, logger=None):
        """初始化导航模拟器
        
        Args:
            logger: 日志函数，默认为 print
        """
        self._logger = logger or print
        self._current_pose = None
        self._goal_pose = None
        
        # 回调函数
        self.on_goal_reached = None
        self.on_goal_failed = None

    def navigate_to_pose_async(self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map') -> bool:
        """异步导航到指定位姿（核心方法）
        
        模拟导航：立即返回成功，不实际移动。
        
        Args:
            x: X坐标（米）
            y: Y坐标（米）
            yaw: 偏航角（弧度）
            frame_id: 坐标系
        
        Returns:
            True 表示导航成功
        """
        self._goal_pose = (x, y, yaw)
        self._current_pose = (x, y, yaw)
        
        self._logger(f"📍 [DummyNav] 导航到: ({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°")
        
        # 模拟导航完成，触发回调
        if self.on_goal_reached:
            self._logger("✅ [DummyNav] 导航到达！")
            self.on_goal_reached()
        
        return True

    def get_current_pose(self):
        """获取当前位姿"""
        return self._current_pose

    def cancel_navigation(self):
        """取消导航"""
        self._logger("🛑 [DummyNav] 取消导航")
        self._goal_pose = None
        return True


def main():
    """简单测试"""
    def log(msg):
        print(f"[Test] {msg}")
    
    nav = DummyNavigator(logger=log)
    
    # 设置回调
    nav.on_goal_reached = lambda: print("[Callback] 到达目标！")
    
    # 测试导航
    nav.navigate_to_pose_async(1.0, 2.0, 0.5)
    print(f"当前位置: {nav.get_current_pose()}")


if __name__ == "__main__":
    main()
