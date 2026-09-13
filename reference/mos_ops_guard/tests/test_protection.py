import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

from mos_ops_guard.protection import protect, recover


def configuration() -> dict:
    enabled = {"enabled": True, "argv": ["/bin/true"]}
    actions = {
        name: dict(enabled)
        for name in (
            "lock_tasks",
            "unlock_tasks",
            "cancel_navigation",
            "cancel_arm",
            "cancel_scheduler",
            "cancel_state_machine",
            "zero_base",
            "zero_arm",
            "hardware_estop",
            "clear_software_protection",
            "physical_estop_released",
        )
    }
    return {
        "command_timeout_seconds": 2,
        "event_log": {"max_bytes": 4096, "backups": 2},
        "recovery": {"allow_onsite_estop_fallback": True},
        "actions": actions,
        "common_cancel_actions": [
            "lock_tasks",
            "cancel_navigation",
            "cancel_arm",
            "cancel_scheduler",
            "cancel_state_machine",
            "zero_base",
        ],
        "p0_actions": ["hardware_estop", "zero_base", "zero_arm"],
        "managed_subsystems": {
            name: {"stop_action": dict(enabled), "start_action": dict(enabled)}
            for name in ("navigation", "base", "arm", "vision", "scheduler")
        },
    }


class ProtectionTest(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root / "config").mkdir()
        (self.root / "config" / "protection.yaml").write_text(
            yaml.safe_dump(configuration()), encoding="utf-8"
        )
        self.environment = patch.dict(
            "os.environ", {"MOS_OPS_GUARD_ROOT": str(self.root)}
        )
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.publisher = patch(
            "mos_ops_guard.protection._publish_alarm_best_effort", return_value=None
        )
        self.publisher.start()
        self.addCleanup(self.publisher.stop)

    def test_same_protection_is_idempotent_and_recovery_does_not_replay_task(self) -> None:
        first = protect("p1", reason="test", operator="alice", scopes=["vision"])
        self.assertFalse(first["idempotent"])
        self.assertEqual(first["state"]["event_count"], 1)
        second = protect("p1", reason="again", operator="bob", scopes=["vision"])
        self.assertTrue(second["idempotent"])
        self.assertEqual(second["state"]["event_count"], 1)
        result = recover(operator="alice", onsite_confirmed=True)
        self.assertEqual(result["state"]["state"], "NORMAL")
        self.assertFalse(result["state"]["old_task_replayed"])

    def test_lower_level_cannot_override_higher_level(self) -> None:
        result = protect("p0", reason="test", operator="alice")
        action_names = [item["action"] for item in result["actions"]]
        self.assertEqual(action_names[0], "hardware_estop")
        self.assertEqual(action_names.count("zero_base"), 1)
        with self.assertRaises(RuntimeError):
            protect("p2", reason="downgrade", operator="alice")

    def test_dry_run_does_not_create_state(self) -> None:
        result = protect("p2", reason="test", operator="alice", dry_run=True)
        self.assertTrue(result["dry_run"])
        runtime = self.root / "runtime"
        self.assertFalse(runtime.exists() and any(runtime.iterdir()))

    def test_recovery_dry_run_does_not_change_active_state(self) -> None:
        protect("p1", reason="test", operator="alice", scopes=["vision"])
        result = recover(operator="alice", onsite_confirmed=True, dry_run=True)
        self.assertTrue(result["dry_run"])
        state = yaml.safe_load(
            (self.root / "runtime" / "latest_protection.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(state["state"], "P1")


if __name__ == "__main__":
    unittest.main()
