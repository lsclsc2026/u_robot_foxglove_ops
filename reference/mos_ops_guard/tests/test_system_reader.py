import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from mos_ops_guard.system_reader import read_text


class SystemReaderTest(unittest.TestCase):
    def test_jetson_virtual_sensor_type_error_is_treated_as_unavailable(self) -> None:
        with patch.object(Path, "read_text", side_effect=TypeError("virtual sensor")):
            self.assertIsNone(read_text(Path("/sys/class/thermal/fake/temp")))


if __name__ == "__main__":
    unittest.main()
