import unittest

from mos_ops_guard.ros_reader import AVERAGE_RATE_RE


class RosReaderTest(unittest.TestCase):
    def test_ros2_topic_hz_output_is_parsed(self) -> None:
        output = "average rate: 199.511\n\tmin: 0.002s max: 0.009s\n"
        self.assertEqual(AVERAGE_RATE_RE.findall(output), ["199.511"])


if __name__ == "__main__":
    unittest.main()
