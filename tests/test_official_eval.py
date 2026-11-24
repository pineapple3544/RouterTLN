"""Controller/evaluation orchestration contract tests; no ROS runtime needed."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("official_eval", ROOT / "scripts/official_eval.py")
official = importlib.util.module_from_spec(spec)
spec.loader.exec_module(official)


class OfficialEvalTests(unittest.TestCase):
    def test_controller_command_only_changes_log_destination(self):
        setup = {"binary": "/build/el_controller"}
        frozen = Path("/tmp/frozen.yaml")
        a = official.controller_cmd(setup, frozen, Path("/tmp/obs1"))
        b = official.controller_cmd(setup, frozen, Path("/tmp/obs5"))
        self.assertEqual(a[:-1], b[:-1])
        self.assertEqual(a[1:-1], ["--ros-args", "--params-file", str(frozen), "-p"])
        self.assertEqual(a[-1], "metrics:=/tmp/obs1/control.csv")

    def test_fingerprints_reject_profile_or_model_change(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            original, frozen, model = [root / name for name in ("source.yaml", "copy.yaml", "model.pt")]
            original.write_text("profile")
            frozen.write_text("profile")
            model.write_bytes(b"weights")
            expected = official.sha(original)
            setup = {"fingerprints": {str(original): expected, str(model): official.sha(model)}}
            official.check_frozen(setup, frozen, expected)
            model.write_bytes(b"changed")
            with self.assertRaises(RuntimeError):
                official.check_frozen(setup, frozen, expected)
            model.write_bytes(b"weights")
            frozen.write_text("changed")
            with self.assertRaises(RuntimeError):
                official.check_frozen(setup, frozen, expected)

    def test_profiles_use_same_map_and_weights_and_official_clock(self):
        import yaml
        def params(name):
            return yaml.safe_load((ROOT / "config/official" / name).read_text())["/el_native_controller"]["ros__parameters"]
        a, b = params("v1.yaml"), params("v2.yaml")
        for p in (a, b):
            self.assertTrue(p["official_simulator"])
            self.assertFalse(p["local_steady_sim_clock"])
            self.assertEqual(p["stop_after_laps"], 0)
            self.assertTrue(p["lap_planning"])
            self.assertEqual(p["deadline_ms"], 25.0)
        for key in ("map_dir", "race_checkpoint", "avoid_checkpoint", "inference_device",
                    "use_sim_time", "max_speed"):
            self.assertEqual(a[key], b[key])
        self.assertFalse(a["hierarchical_v2"])
        self.assertTrue(b["hierarchical_v2"])
        self.assertTrue(b["v2_policy_first"])

    def test_json_output(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / "result.json"
            official.write_json(p, {"status": "not validated"})
            self.assertEqual(json.loads(p.read_text()), {"status": "not validated"})


if __name__ == "__main__":
    unittest.main()
