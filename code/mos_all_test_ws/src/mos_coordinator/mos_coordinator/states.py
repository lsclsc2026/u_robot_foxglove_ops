#!/usr/bin/env python3
"""任务状态定义：两层状态机。

Phase — 整体业务阶段（找满 / 放满 / 找空 / 放空）
Step  — 阶段内子步骤（巡航 / 观察 / 动作 / 完成 / 回）
        其中「回」本质也是巡航（导航回起点），完成后通常回到 Cruise/Observe。
"""

from enum import Enum


class Phase(Enum):
    """整体业务阶段。"""
    IDLE = "IDLE"
    SearchFull = "SearchFull"      # 找满车
    PutFull = "PutFull"            # 放满车
    SearchEmpty = "SearchEmpty"    # 找空车
    PutEmpty = "PutEmpty"          # 放空车
    ERROR = "ERROR"


class Step(Enum):
    """业务阶段内的子步骤。"""
    Cruise = "Cruise"      # 巡航 / 导航
    Observe = "Observe"    # 观察 / 视觉判断
    Act = "Act"            # 动作（抓取 / 放置）
    Done = "Done"          # 本阶段完成 → 切下一 Phase
    GoBack = "GoBack"      # 回（也是巡航：导航回起点后继续）


class TaskState(Enum):
    """任务执行状态枚举 - 简化版本"""
    IDLE = "idle"                        # 空闲
    SearchFull = "SearchFull"            # 搜索满载小车（AB区间）
    SearchPutFullPos = "SearchPutFullPos"            # 搜索满载小车放置位置
    SearchEmpty = "SearchEmpty"          # 搜索空载小车（DE区间）- 找空阶段
    SearchPutEmptyPos = "SearchPutEmptyPos"          # 搜索空载小车放置位置
    APPROACHING = "approaching"          # 接近目标小车
    ALIGNING = "aligning"                # 底盘对准中
    POSITIONING = "positioning"          # 等待把手检测/准备抓取
    GRASPING = "grasping"                # 抓取中
    TRANSPORTING = "transporting"        # 运输中（手持小车前往卸货点）
    RELEASING = "releasing"              # 释放小车中
    GoBack = "GoBack"                    # 返回
    ERROR = "error"                      # 错误状态
