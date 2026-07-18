import os
import unittest
from unittest.mock import patch

from tools import agentctl


class AgentctlTest(unittest.TestCase):
    def test_target_id_defaults_to_environment(self):
        with patch.dict(os.environ, {"GAMEUAV_UAV_ID": "uav0"}):
            args = agentctl.build_parser(["health"]).parse_args(["health"])

        self.assertEqual(args.target_id, "uav0")

    def test_target_id_defaults_to_hostname(self):
        with patch.dict(os.environ, {"GAMEUAV_UAV_ID": ""}):
            with patch("tools.agentctl.socket.gethostname", return_value="uav0"):
                args = agentctl.build_parser(["health"]).parse_args(["health"])

        self.assertEqual(args.target_id, "uav0")


if __name__ == "__main__":
    unittest.main()
