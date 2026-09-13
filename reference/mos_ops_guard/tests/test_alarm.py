import unittest

from mos_ops_guard.alarm import highest_alarm, make_alarm
from mos_ops_guard.status_mux import AlarmLatch


class AlarmTest(unittest.TestCase):
    def test_latch_is_source_scoped_and_deduplicates(self) -> None:
        latch = AlarmLatch()
        first = make_alarm(
            "disk",
            active=True,
            source="watchdog",
            item="disk",
        )
        self.assertEqual(latch.update(first), (True, False))
        duplicate = dict(first)
        duplicate["timestamp"] = "later"
        duplicate["details"] = {"age_seconds": 100}
        self.assertEqual(latch.update(duplicate), (False, False))
        other = make_alarm(
            "disk",
            active=True,
            source="protection",
            severity="critical",
            item="protection",
        )
        self.assertEqual(latch.update(other), (True, False))
        self.assertEqual(highest_alarm(latch.active.values())["source"], "protection")
        clear_first = dict(first, active=False)
        self.assertEqual(latch.update(clear_first), (True, False))
        clear_other = dict(other, active=False)
        self.assertEqual(latch.update(clear_other), (True, True))


if __name__ == "__main__":
    unittest.main()
