import json
import tempfile
import unittest
from pathlib import Path

from mos_ops_guard.storage import append_rotating_jsonl, atomic_write_json, read_json


class StorageTest(unittest.TestCase):
    def test_atomic_json_and_bounded_rotation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            state = root / "state.json"
            atomic_write_json(state, {"中文": "正常"})
            self.assertEqual(read_json(state, {})["中文"], "正常")

            log = root / "events.jsonl"
            for index in range(30):
                append_rotating_jsonl(
                    log,
                    {"index": index, "payload": "x" * 40},
                    max_bytes=180,
                    backups=3,
                )
            self.assertTrue(log.exists())
            self.assertTrue(log.with_name("events.jsonl.1").exists())
            self.assertFalse(log.with_name("events.jsonl.4").exists())
            for path in root.glob("events.jsonl*"):
                for line in path.read_text(encoding="utf-8").splitlines():
                    json.loads(line)


if __name__ == "__main__":
    unittest.main()
