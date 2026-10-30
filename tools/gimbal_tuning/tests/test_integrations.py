import copy
from dataclasses import replace
import importlib.metadata
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from gimbal_tuning.config import Configuration, wire_bounds, STAGE_FIELDS
from gimbal_tuning.optimization import BayesianProposer, materialize
from gimbal_tuning.recording import VofaForwarder
from gimbal_tuning.protocol import Telemetry
from gimbal_tuning.manager import ExperimentManager
from gimbal_tuning.analysis import replay
from support import fast_config, EXAMPLES

try:
    REAL_BO = importlib.metadata.version("bayesian-optimization") == "3.1.0"
except importlib.metadata.PackageNotFoundError:
    REAL_BO = False


class IntegrationTests(unittest.TestCase):
    def test_stages_schema_conditional_acceptance_and_binary32_bounds(self):
        config = fast_config()
        for change in ("speed_acceptance", "stage", "fixed_gains", "mode", "speed_units", "score_version"):
            raw = copy.deepcopy(config.raw)
            c = raw["gate_c"]
            if change == "speed_acceptance":
                c["speed_loop_accepted"] = False
            elif change == "stage":
                c["stage"] = "all_gains"
            elif change == "fixed_gains":
                c["search"]["speed_p"] = [99, 101]
            elif change == "mode":
                c["scenarios"][0]["mode"] = 1
            elif change == "speed_units":
                del c["performance"]["final_error_rad_s"]
            else:
                c["scoring_version"] = "constraints_first_v1"
            with self.subTest(change=change), self.assertRaises(ValueError):
                Configuration(raw).validate("C")
        raw = copy.deepcopy(config.raw)
        raw["environment"] = "hardware"
        raw["baseline"]["algorithm"] = raw["candidate"]["algorithm"] = 1
        with self.assertRaisesRegex(ValueError, "trial authorization"):
            Configuration(raw).validate("B", hardware=True)
        # A prior acceptance cannot substitute for explicit diagnostic authorization.
        raw["confirmations"]["conditional_baseline_accepted"] = True
        with self.assertRaisesRegex(ValueError, "trial authorization"):
            Configuration(raw).validate("B", hardware=True)
        raw["confirmations"]["conditional_baseline_accepted"] = False
        raw["confirmations"]["conditional_trial_authorized"] = True
        Configuration(raw).validate("B", hardware=True)
        with self.assertRaisesRegex(ValueError, "independently"):
            Configuration(raw).validate("C", hardware=True)
        raw["confirmations"]["conditional_baseline_accepted"] = True
        Configuration(raw).validate("C", hardware=True)
        raw["gate_c"]["baseline_accepted"] = False
        with self.assertRaisesRegex(ValueError, "baseline_accepted"):
            Configuration(raw).validate("C", hardware=True)
        with self.assertRaises(ValueError):
            wire_bounds(0.1, 0.1)
        lo, hi = wire_bounds(0.1, 0.2)
        self.assertGreaterEqual(lo, 0.1)
        self.assertLessEqual(hi, 0.2)

    def test_bidirectional_noncollapsed_excitation_and_u32_batch_product(self):
        config = fast_config()
        for mutation in ("zero", "collapsed", "one_direction", "product"):
            raw = copy.deepcopy(config.raw)
            c = raw["gate_c"]
            if mutation == "zero":
                c["scenarios"][0]["target"] = c["scenarios"][0]["origin"]
            elif mutation == "collapsed":
                c["scenarios"][0].update(origin=0.1, target=0.1000000001)
            elif mutation == "one_direction":
                c["scenarios"] = c["scenarios"][:1]
            else:
                c["budget"]["max_candidates"] = 0xffffffff
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                Configuration(raw).validate("C")
        raw = copy.deepcopy(fast_config("speed_pi").raw)
        raw["gate_c"]["scenarios"][0]["target"] = 1e-50
        with self.assertRaises(ValueError):
            Configuration(raw).validate("C")
        # A deliberate zero-step remains a Gate-B diagnostic, never a BO observation.
        raw = copy.deepcopy(config.raw)
        raw["trial"]["target"] = raw["trial"]["origin"]
        Configuration(raw).validate("B")

    def test_proposals_cannot_change_stage_or_algorithm(self):
        for stage in STAGE_FIELDS:
            config = fast_config(stage)
            fields = STAGE_FIELDS[stage]
            valid = {n: sum(config.raw["gate_c"]["search"][n]) / 2 for n in fields}
            params = materialize(config, valid, 8)
            self.assertEqual(params.version, 8)
            self.assertEqual(params.algorithm, config.baseline.algorithm)
            fixed = set(("pos_p", "pos_d", "speed_p", "speed_i")) - set(fields)
            for n in fixed:
                self.assertEqual(getattr(params, n), getattr(type(config.baseline).unpack(config.baseline.pack()), n))
            for proposal in (dict(valid, algorithm=1), {fields[0]: float("nan"), fields[1]: 0},
                             {fields[0]: True, fields[1]: 0}, {fields[0]: 1e8, fields[1]: 0}):
                with self.assertRaises(ValueError):
                    materialize(config, proposal, 9)

    def test_vofa_justfloat_loopback_best_effort(self):
        class Socket:
            def __init__(self):
                self.packet = None
                self.fail = False
            def setblocking(self, value):
                self.nonblocking = not value
            def sendto(self, packet, endpoint):
                if self.fail:
                    raise BlockingIOError("disconnected/full UDP")
                self.packet, self.endpoint = packet, endpoint
            def close(self):
                pass
        socket = Socket()
        with patch("gimbal_tuning.recording.socket.socket", return_value=socket):
            forwarder = VofaForwarder(port=1347)
            row = replace(Telemetry.unpack(bytes(124)), target_pos=0.5)
            forwarder.send(row)
            self.assertTrue(socket.nonblocking)
            self.assertEqual(socket.packet[-4:], b"\x00\x00\x80\x7f")
            self.assertEqual(struct.unpack("<f", socket.packet[:4])[0], 0.5)
            self.assertEqual(socket.endpoint, ("127.0.0.1", 1347))
            socket.fail = True
            forwarder.send(row)
            self.assertEqual(forwarder.dropped, 1)
            forwarder.close()
        for host in ("192.0.2.1", "0.0.0.0", "example.com"):
            with self.assertRaises(ValueError):
                VofaForwarder(host=host)

    def test_cli_ambiguous_hardware_flags_rejected_before_open(self):
        cwd = EXAMPLES.parent
        environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        for flags in (["--simulate", "--port", "MOCK"], [], ["--port", "MOCK"]):
            with tempfile.TemporaryDirectory() as directory:
                run = Path(directory) / "run"
                command = [sys.executable, "-m", "gimbal_tuning", "baseline", "--config", str(EXAMPLES / "hardware-template.json"), "--run-dir", str(run)] + flags
                process = subprocess.run(command, cwd=cwd, env=environment, capture_output=True, text=True)
                self.assertEqual(process.returncode, 2)
                self.assertFalse(run.exists())

    @unittest.skipUnless(REAL_BO, "optional real pinned library not installed in stdlib interpreter")
    def test_real_pinned_propose_register_and_seed_reproducibility(self):
        config = fast_config("speed_pi")
        left, right = BayesianProposer(config), BayesianProposer(config)
        with self.assertRaisesRegex(ValueError, "baseline"):
            left.suggest()
        observation = {"valid": True, "utility": -1.1}
        for proposer in (left, right):
            proposer.register(config.baseline, observation)
        a, b = left.suggest(), right.suggest()
        self.assertEqual(a, b)
        params = materialize(config, a, 5)
        left.register(params, {"valid": True, "utility": 0.7})
        self.assertEqual(left.observations, 2)
        with self.assertRaises(ValueError):
            left.register(params, {"valid": False, "utility": 0})

    @unittest.skipUnless(REAL_BO, "optional real pinned library not installed in stdlib interpreter")
    def test_real_optimizer_manager_and_offline_replay_proposal(self):
        config = fast_config("speed_pi")
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory) / "run"
            result = ExperimentManager(config, run, simulated_tick_s=0.00005).run_search()
            self.assertEqual(result.get("termination"), "candidate_count", result)
            self.assertTrue(result["zero_confirmed"])
            offline = replay(run, propose=True)
            self.assertEqual(len(offline["observations"]), 2)
            self.assertIn("offline_next_proposal", offline)
            self.assertFalse(offline["motion_resumed"])


if __name__ == "__main__":
    unittest.main()
