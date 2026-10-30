import copy
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
from gimbal_tuning.config import Configuration, CONFIRMATIONS
from gimbal_tuning.protocol import Parameters, Trial

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.config = Configuration.load(EXAMPLES / "simulation.json")

    def check_bad(self, section, key, values, gate="B"):
        for value in values:
            with self.subTest(section=section, key=key, value=value):
                raw = copy.deepcopy(self.config.raw)
                raw[section][key] = value
                with self.assertRaises(ValueError):
                    Configuration(raw).validate(gate)

    def test_bundled_examples_match_user_templates(self):
        from gimbal_tuning.cli import EXAMPLES as bundled
        for name in ("simulation.json", "simulation-speed.json", "hardware-template.json"):
            self.assertEqual((EXAMPLES / name).read_bytes(), (bundled / name).read_bytes())

    def test_examples_and_hardware_refusal(self):
        self.config.validate("C")
        with self.assertRaisesRegex(ValueError, "simulation"):
            self.config.validate("B", hardware=True)
        hardware = Configuration.load(EXAMPLES / "hardware-template.json")
        hardware.validate("A", hardware=True)
        for gate in ("B", "C"):
            with self.assertRaises(ValueError):
                hardware.validate(gate, hardware=True)

    def test_every_confirmation_required(self):
        for name in CONFIRMATIONS:
            self.check_bad("confirmations", name, [None, 1] if name in ("conditional_trial_authorized", "conditional_baseline_accepted") else [False, None, 1])

    def test_every_safety_field_required(self):
        for name in self.config.raw["safety"]:
            self.check_bad("safety", name, [None, True, float("nan"), float("inf"), -float("inf")])
            raw = copy.deepcopy(self.config.raw)
            del raw["safety"][name]
            with self.assertRaises(ValueError):
                Configuration(raw).validate("B")

    def test_numeric_constraints(self):
        for key, values in {
            "margin": [-1, 2], "max_speed": [0, -1, 1e7], "max_output": [32768, 0],
            "max_integral": [0, 1001, 1e-50], "prepare_speed": [3], "pos_max": [-2],
            "gain_min_pos_p": [-1, 100], "gain_max_speed_i": [-1],
            "heartbeat_timeout_ms": [0, 60001, 1.5], "feedback_timeout_ms": [60001],
            "batch_timeout_ms": [1800001], "prepare_timeout_ms": [1800001],
            "settle_hold_ms": [8001], "telemetry_period_ms": [4],
            "max_control_gap_us": [999, 100001], "max_trial_ms": [60001],
        }.items():
            self.check_bad("safety", key, values)

    def test_params_trial_identity_strict(self):
        for section, key, values in [
            ("baseline", "version", [0, True]), ("candidate", "version", [1]),
            ("candidate", "algorithm", [True, 2, 1]), ("baseline", "speed_i", [-1, True, None]),
            ("trial", "mode", [2, True]), ("trial", "origin", [1.0, True]),
            ("trial", "target", [1.0]), ("trial", "duration_ms", [0, 3001, True]),
            ("identity", "firmware", [None, True, 2]), ("identity", "angle_unit", ["deg"]),
        ]:
            self.check_bad(section, key, values)
        with self.assertRaises(ValueError):
            replace(self.config.trial, mode=1, target=3).validate(self.config.safety)

    def test_gate_c_all_acceptances_bounds_and_budgets(self):
        for name in ("baseline_accepted", "preparation_accepted", "performance_accepted", "search_accepted", "budget_accepted"):
            self.check_bad("gate_c", name, [False, None, 1], gate="C")
        for group, key, value in [
            ("performance", "final_error_rad", True), ("performance", "max_missing_fraction", 1),
            ("performance", "hold_ms", 901), ("search", "speed_i", [-1, 1]),
            ("search", "pos_p", [9, 10]), ("budget", "max_total_ms", 60001),
            ("budget", "repeats", 0), ("budget", "max_trial_ms", 3001),
            ("budget", "termination_rule", "retry_on_fault"),
        ]:
            raw = copy.deepcopy(self.config.raw)
            raw["gate_c"][group][key] = value
            with self.subTest(group=group, key=key), self.assertRaises(ValueError):
                Configuration(raw).validate("C")
        self.check_bad("candidate", "pos_p", [20], gate="C")
        for key, values in {"scenarios": [[]], "random_seed": [True], "firmware": [2], "parameter_units": [None]}.items():
            self.check_bad("gate_c", key, values, gate="C")

    def test_json_nonfinite_and_duplicate_keys(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            for text in ('{"x": NaN}', '{"x": Infinity}', '{"x": 1e999}', '{"x": 1, "x": 2}'):
                path.write_text(text)
                with self.subTest(text=text), self.assertRaises(ValueError):
                    Configuration.load(path)


if __name__ == "__main__":
    unittest.main()
