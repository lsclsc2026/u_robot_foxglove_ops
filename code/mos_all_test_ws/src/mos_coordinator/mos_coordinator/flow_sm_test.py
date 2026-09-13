#!/usr/bin/env python3
"""本机测试：仅验证 Phase / Step 流转（不涉及导航、视觉、抓取等业务逻辑）。

用法::

    python3 -m mos_coordinator.flow_sm_test
    # 或（若已安装入口）
    ros2 run mos_coordinator flow_sm_test
"""

from __future__ import annotations

import sys
from typing import List, Optional, Tuple

from mos_coordinator.states import Phase, Step


State = Tuple[Phase, Step]


def label(phase: Phase, step: Step) -> str:
    return f"{phase.value}/{step.value}"


class FlowSm:
    """纯流转引擎：只维护 phase/step，按事件推进。"""

    # Phase 完成(Done)后的下一业务阶段
    NEXT_PHASE = {
        Phase.SearchFull: Phase.PutFull,
        Phase.PutFull: Phase.SearchEmpty,
        Phase.SearchEmpty: Phase.PutEmpty,
        Phase.PutEmpty: Phase.SearchFull,  # 放空结束经 GoBack 后由事件切回找满
    }

    def __init__(self):
        self.phase = Phase.IDLE
        self.step = Step.Cruise
        self.trace: List[str] = []
        self._record()

    def _record(self):
        self.trace.append(label(self.phase, self.step))

    def start(self, phase: Phase = Phase.SearchFull):
        """任务开始 → 进入某 Phase 的 Observe（或 Cruise）。"""
        self.phase = phase
        self.step = Step.Observe
        self._record()

    def on_cruise_done(self):
        """巡航/导航到达 → 观察。"""
        assert self.step == Step.Cruise, f"期望 Cruise, 实际 {self.step}"
        self.step = Step.Observe
        self._record()

    def on_observe_need_move(self):
        """观察后需要再去下一点 / 抓车位 → 巡航。"""
        assert self.step == Step.Observe, f"期望 Observe, 实际 {self.step}"
        self.step = Step.Cruise
        self._record()

    def on_observe_ready_act(self):
        """观察通过，开始动作。"""
        assert self.step == Step.Observe, f"期望 Observe, 实际 {self.step}"
        self.step = Step.Act
        self._record()

    def on_act_success(self):
        """动作成功 → 完成。"""
        assert self.step == Step.Act, f"期望 Act, 实际 {self.step}"
        self.step = Step.Done
        self._record()

    def on_act_fail(self):
        """动作失败 → 回。"""
        assert self.step == Step.Act, f"期望 Act, 实际 {self.step}"
        self.step = Step.GoBack
        self._record()

    def on_observe_exhausted(self):
        """观察耗尽（搜完无目标）→ 回。"""
        assert self.step == Step.Observe, f"期望 Observe, 实际 {self.step}"
        self.step = Step.GoBack
        self._record()

    def on_done(self):
        """阶段完成 → 下一 Phase，从 Cruise 开始。"""
        assert self.step == Step.Done, f"期望 Done, 实际 {self.step}"
        nxt = self.NEXT_PHASE.get(self.phase)
        if nxt is None:
            raise AssertionError(f"{self.phase} 无下一 Phase")
        self.phase = nxt
        self.step = Step.Cruise
        self._record()

    def on_put_empty_done(self):
        """放空完成 → 回（再回找满）。"""
        assert self.phase == Phase.PutEmpty and self.step == Step.Done
        self.step = Step.GoBack
        self._record()

    def on_goback_arrived(self):
        """回起点到达 → 找满 / Observe。"""
        assert self.step == Step.GoBack, f"期望 GoBack, 实际 {self.step}"
        self.phase = Phase.SearchFull
        self.step = Step.Observe
        self._record()


def _assert_trace(trace: List[str], expected: List[str], name: str):
    if trace != expected:
        raise AssertionError(
            f"[{name}]\n  expected: {' → '.join(expected)}\n"
            f"  actual:   {' → '.join(trace)}"
        )


def scenario_search_full_happy() -> List[str]:
    """找满成功：Observe→Cruise→Observe→Act→Done→PutFull。"""
    sm = FlowSm()
    sm.start(Phase.SearchFull)          # SearchFull/Observe
    sm.on_observe_need_move()           # → Cruise（去下一点）
    sm.on_cruise_done()                 # → Observe
    sm.on_observe_ready_act()           # → Act
    sm.on_act_success()                 # → Done
    sm.on_done()                        # → PutFull/Cruise
    expected = [
        "IDLE/Cruise",
        "SearchFull/Observe",
        "SearchFull/Cruise",
        "SearchFull/Observe",
        "SearchFull/Act",
        "SearchFull/Done",
        "PutFull/Cruise",
    ]
    _assert_trace(sm.trace, expected, "search_full_happy")
    return sm.trace


def scenario_search_full_miss_goback() -> List[str]:
    """找满失败：Observe 耗尽 → GoBack → 回找满 Observe。"""
    sm = FlowSm()
    sm.start(Phase.SearchFull)
    sm.on_observe_need_move()
    sm.on_cruise_done()
    sm.on_observe_exhausted()           # → GoBack
    sm.on_goback_arrived()              # → SearchFull/Observe
    expected = [
        "IDLE/Cruise",
        "SearchFull/Observe",
        "SearchFull/Cruise",
        "SearchFull/Observe",
        "SearchFull/GoBack",
        "SearchFull/Observe",
    ]
    _assert_trace(sm.trace, expected, "search_full_miss_goback")
    return sm.trace


def scenario_full_cycle() -> List[str]:
    """完整业务环：找满→放满→找空→放空→回→找满。"""
    sm = FlowSm()
    # 找满
    sm.start(Phase.SearchFull)
    sm.on_observe_ready_act()
    sm.on_act_success()
    sm.on_done()                        # PutFull/Cruise
    # 放满
    sm.step = Step.Done
    sm._record()
    sm.on_done()                        # SearchEmpty/Cruise
    # 找空
    sm.step = Step.Observe
    sm._record()
    sm.on_observe_ready_act()
    sm.on_act_success()
    sm.on_done()                        # PutEmpty/Cruise
    # 放空 → 回 → 找满
    sm.step = Step.Done
    sm._record()
    sm.on_put_empty_done()              # PutEmpty/GoBack
    sm.on_goback_arrived()              # SearchFull/Observe

    expected = [
        "IDLE/Cruise",
        "SearchFull/Observe",
        "SearchFull/Act",
        "SearchFull/Done",
        "PutFull/Cruise",
        "PutFull/Done",
        "SearchEmpty/Cruise",
        "SearchEmpty/Observe",
        "SearchEmpty/Act",
        "SearchEmpty/Done",
        "PutEmpty/Cruise",
        "PutEmpty/Done",
        "PutEmpty/GoBack",
        "SearchFull/Observe",
    ]
    _assert_trace(sm.trace, expected, "full_cycle")
    return sm.trace


def scenario_act_fail_goback() -> List[str]:
    """动作失败 → GoBack → 找满。"""
    sm = FlowSm()
    sm.start(Phase.SearchFull)
    sm.on_observe_ready_act()
    sm.on_act_fail()
    sm.on_goback_arrived()
    expected = [
        "IDLE/Cruise",
        "SearchFull/Observe",
        "SearchFull/Act",
        "SearchFull/GoBack",
        "SearchFull/Observe",
    ]
    _assert_trace(sm.trace, expected, "act_fail_goback")
    return sm.trace


def main(args=None):
    scenarios = [
        ("search_full_happy", scenario_search_full_happy),
        ("search_full_miss_goback", scenario_search_full_miss_goback),
        ("act_fail_goback", scenario_act_fail_goback),
        ("full_cycle", scenario_full_cycle),
    ]
    failures = []
    for name, fn in scenarios:
        print(f"\n===== {name} =====")
        try:
            trace = fn()
            print("trace:", " → ".join(trace))
            print("PASS")
        except Exception as e:
            print(f"FAIL: {e}")
            failures.append(name)

    if failures:
        print(f"\nFAILED: {failures}")
        raise SystemExit(1)
    print("\nALL PASSED")


if __name__ == "__main__":
    main(sys.argv[1:])
