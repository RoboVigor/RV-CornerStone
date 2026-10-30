import copy
from dataclasses import replace, asdict
import json
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from gimbal_tuning.config import Configuration
from gimbal_tuning.protocol import Frame, Message as M, State as S, Fault as F, Telemetry, Status, StreamDecoder
from gimbal_tuning.simulator import SimulatedDevice, SimulatedTransport
from gimbal_tuning.manager import ExperimentManager
from gimbal_tuning.recording import RunLog
from gimbal_tuning.analysis import replay, compare_baselines
from gimbal_tuning.transport import ProtocolClient, SerialTransport
from support import fast_config


class Proposer:
    def __init__(self, config, delay=0):
        self.config, self.delay = config, delay
        self.registered, self.suggestions = [], 0
        self.manager = None
        self.heartbeat_progress = None

    def register(self, parameters, outcome):
        self.registered.append((parameters, outcome))

    def suggest(self):
        if not self.registered:
            raise AssertionError("suggest before baseline registration")
        self.suggestions += 1
        before = self.manager.owner.heartbeat_count if self.manager else 0
        time.sleep(self.delay)
        after = self.manager.owner.heartbeat_count if self.manager else 0
        self.heartbeat_progress = after - before
        fields = ("speed_p", "speed_i") if self.config.raw["gate_c"]["stage"] == "speed_pi" else ("pos_p", "pos_d")
        return {name: sum(self.config.raw["gate_c"]["search"][name]) / 2 for name in fields}


class FaultDevice(SimulatedDevice):
    def __init__(self, injection, **kwargs):
        super().__init__(**kwargs)
        self.injection = injection
        self.injected = False
        self.held = None
        self.late_fault = False
        self.stop_writes = 0
        self.starts = 0

    def _command(self, frame):
        if frame.type == M.STOP:
            self.stop_writes += 1
        if frame.type == M.START_TRIAL:
            self.starts += 1
        super()._command(frame)

    def _emit(self, kind, payload, seq=0):
        if kind == M.ACK and struct.unpack("<I", payload[:4])[0] == M.STOP:
            if self.injection == "stop_loss":
                return
            if self.injection == "stale_stop":
                super()._emit(kind, payload, seq)
                self.outbox[-1] = replace(self.outbox[-1], boot_id=999)
                return
        if kind == M.STATUS and self.state == S.COMPLETE_ZERO and self.injection == "bad_zero_status":
            payload = replace(Status.unpack(payload), fault=F.TRANSPORT).pack()
        if kind == M.RESULT and self.injection == "late_can":
            self.late_fault = True
        if kind == M.TELEMETRY:
            row = Telemetry.unpack(payload)
            if row.state == S.RUN:
                if self.injection == "reorder":
                    if self.held is not None:
                        super()._emit(kind, payload, seq)
                        super()._emit(kind, self.held, seq)
                        self.held = None
                        self.injected = True
                        return
                    if not self.injected:
                        self.held = payload
                        return
                if not self.injected and row.sample_seq > self.run_start_seq:
                    if self.injection in ("nonfinite", "stop_loss", "stale_stop"):
                        payload = replace(row, pos0=float("nan")).pack()
                        self.injected = True
                    elif self.injection == "drop" and row.sample_seq >= self.run_start_seq + 30:
                        self.injected = True
                        return
        super()._emit(kind, payload, seq)

    def drain(self):
        if self.injection == "ack_loss":
            self.outbox = type(self.outbox)(f for f in self.outbox if not (f.type == M.ACK and struct.unpack("<I", f.payload[:4])[0] == M.START_TRIAL))
        return super().drain()

    def _step(self):
        if self.late_fault:
            self.late_fault = False
            self._fault(F.TRANSPORT)
            return False
        return super()._step()


class ManagerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def manager(self, config=None, device=None, name="run", **kwargs):
        config = config or fast_config()
        if device is not None:
            kwargs["transport_factory"] = lambda: SimulatedTransport(device)
        return ExperimentManager(config, self.root / name, simulated_tick_s=0.00005, **kwargs)

    def test_single_durable_replay_and_second_explicit_same_device_session(self):
        config = fast_config()
        device = SimulatedDevice(88)
        first = self.manager(config, device, "first").run_single(baseline=True)
        self.assertEqual(first["classification"], "COMPLETE", first)
        second = self.manager(config, device, "second").run_single()
        self.assertEqual(second["classification"], "COMPLETE", second)
        self.assertGreater(second["effective_baseline"]["version"], first["trials"][0]["parameters"]["version"])
        self.assertEqual(first["effective_baseline"]["pos_p"], second["effective_baseline"]["pos_p"])
        self.assertNotEqual(first["trials"][0]["expected"]["session"], second["trials"][0]["expected"]["session"])
        self.assertTrue(second["zero_confirmed"])
        a = replay(self.root / "second")
        self.assertTrue(a["integrity_valid"])
        self.assertTrue(a["raw_framing_valid"])
        self.assertTrue(a["trials"][0]["score"]["valid"])
        self.assertIsNone(a["trials"][0]["score"]["utility"])
        self.assertIn("integral_abs_max", a["trials"][0]["score"]["metrics"]["diagnostics"])
        self.assertEqual(a, replay(self.root / "second"))
        self.assertFalse(second["baseline_accepted"])
        self.assertEqual(device.output, 0)
        self.assertEqual(device.batch_id, 0)

    def test_search_baseline_first_fixed_stage_repeats_and_replay(self):
        for stage in ("speed_pi", "position_pd"):
            config = fast_config(stage, repeats=2)
            proposer = Proposer(config)
            manager = self.manager(config, name=stage)
            proposer.manager = manager
            result = manager.run_search(proposer)
            self.assertEqual(result["classification"], "BUDGET_EXHAUSTED", result)
            self.assertEqual(result.get("termination"), "candidate_count", result)
            self.assertEqual(len(result["observations"]), 2)
            self.assertEqual(len(result["trials"]), 8)
            self.assertEqual(proposer.suggestions, 1)
            self.assertEqual(result["observations"][0]["kind"], "accepted_baseline")
            for index, observation in enumerate(result["observations"]):
                trial = next(t for t in result["trials"] if t["expected"]["trial"]["trial_id"] == observation["trial_ids"][0])
                self.assertEqual(observation["parameters"], trial["parameters"])
                self.assertEqual(asdict(proposer.registered[index][0]), trial["parameters"])
            fixed = ("pos_p", "pos_d") if stage == "speed_pi" else ("speed_p", "speed_i")
            for trial in result["trials"]:
                for name in fixed:
                    self.assertEqual(trial["parameters"][name], result["effective_baseline"][name])
                self.assertEqual(trial["expected"]["trial"]["mode"], int(stage == "speed_pi"))
            self.assertEqual(len(set(t["parameters"]["version"] for t in result["trials"])), 8)
            self.assertEqual(manager.owner.transport.device.active.version, result["trials"][-1]["parameters"]["version"])
            self.assertEqual(manager.owner.transport.device.output, 0)
            best = result["best_candidate"]
            self.assertTrue(best["requires_independent_validation"])
            self.assertFalse(best["publishable"])
            self.assertFalse(best["accepted_speed_loop"])
            offline = replay(self.root / stage)
            self.assertEqual(len(offline["observations"]), 2)
            self.assertEqual(offline, replay(self.root / stage))

    def test_heartbeat_continues_during_slow_optimizer_and_fsync(self):
        class SlowLog(RunLog):
            def checkpoint(self, timeout=10):
                time.sleep(0.04)
                super().checkpoint(timeout)
        config = fast_config("speed_pi")
        proposer = Proposer(config, delay=0.12)
        manager = self.manager(config, log_factory=SlowLog)
        proposer.manager = manager
        result = manager.run_search(proposer)
        self.assertEqual(result.get("termination"), "candidate_count", result)
        self.assertGreater(proposer.heartbeat_progress, 5)
        self.assertGreater(result["heartbeat_count"], 10)
        self.assertLess(result["max_heartbeat_gap_ms"], config.safety.heartbeat_timeout_ms)

    def test_corrupt_run_aborts_and_never_registers(self):
        for injection, classification in [("nonfinite", "TELEMETRY_INVALID"), ("drop", "TELEMETRY_INVALID"),
                                          ("reorder", "TELEMETRY_INVALID"), ("late_can", "PROTECTION"), ("bad_zero_status", "PROTECTION")]:
            with self.subTest(injection=injection):
                config = fast_config()
                config.raw["gate_c"]["performance"]["max_missing_fraction"] = 0
                device = FaultDevice(injection)
                proposer = Proposer(config)
                result = self.manager(config, device, injection).run_search(proposer)
                self.assertEqual(result["classification"], classification, result)
                self.assertEqual(proposer.registered, [])
                self.assertTrue(result["stop_confirmed"])
                self.assertEqual(device.stop_writes, 1)
                self.assertEqual(device.starts, 1)
                self.assertEqual(device.output, 0)
                self.assertEqual(device.batch_id, 0)
                raw = (self.root / injection / "raw.jsonl").read_text()
                self.assertIn('"direction":"rx"', raw)
                if injection == "nonfinite":
                    self.assertIn('"pos0":"nan"', (self.root / injection / "telemetry.jsonl").read_text())

    def test_stop_ack_loss_is_unconfirmed_not_fabricated_zero(self):
        for injection in ("stop_loss", "stale_stop"):
            device = FaultDevice(injection)
            result = self.manager(device=device, name=injection).run_single()
            self.assertFalse(result["stop_confirmed"])
            self.assertFalse(result["zero_confirmed"])
            self.assertTrue(result["requires_manual_estop"])
            self.assertEqual(device.stop_writes, 1)
            self.assertEqual(device.output, 0)  # instrumentation only; host may not claim this

    def test_uncertain_start_ack_is_never_retried(self):
        device = FaultDevice("ack_loss")
        result = self.manager(device=device).run_single()
        self.assertEqual(result["classification"], "COMMUNICATION_INVALID")
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(device.starts, 1)
        self.assertEqual(device.stop_writes, 1)

    def test_time_budget_exhaustion_is_separate_and_partial_never_registered(self):
        config = fast_config("speed_pi")
        config.raw["gate_c"]["budget"]["max_total_ms"] = 100
        device = SimulatedDevice()
        proposer = Proposer(config)
        result = self.manager(config, device).run_search(proposer)
        self.assertEqual(result["classification"], "BUDGET_EXHAUSTED", result)
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(proposer.registered, [])
        self.assertEqual(device.output, 0)

    def test_ctrl_c_and_storage_failure_stop_without_rescue(self):
        config = fast_config()
        manager = self.manager(config)
        proposer = Proposer(config)
        def interrupted():
            raise KeyboardInterrupt()
        proposer.suggest = interrupted
        result = manager.run_search(proposer)
        self.assertEqual(result["classification"], "STOPPED")
        self.assertTrue(result["interrupted"])
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(len(proposer.registered), 1)
        class FailingLog(RunLog):
            def checkpoint(self, timeout=10):
                raise OSError("injected storage failure")
        result = self.manager(config, name="storage", log_factory=FailingLog).run_single()
        self.assertEqual(result["classification"], "COMMUNICATION_INVALID")
        self.assertTrue(result["stop_confirmed"])

    def test_pre_open_failure_does_not_fabricate_zero_confirmation(self):
        with patch("gimbal_tuning.manager.SimulatedTransport") as factory:
            result = self.manager(vofa={"host": "192.0.2.1", "port": 1347}).run_single()
        self.assertFalse(result["zero_confirmed"])
        self.assertIsNone(result["stop_confirmed"])
        factory.assert_not_called()

    def test_hardware_gate_before_open_and_inferred_serial_capability(self):
        config = fast_config()
        with self.assertRaises(ValueError):
            ExperimentManager(config, self.root / "bad", hardware=True, port="MOCK", authorize_motion=True)
        raw = copy.deepcopy(config.raw)
        raw["environment"] = "hardware"
        config = Configuration(raw)
        with self.assertRaises(ValueError):
            ExperimentManager(config, self.root / "bad", hardware=True, port="MOCK")
        class MockSerial(SerialTransport):
            def __init__(self):
                self.backend = SimulatedTransport()
            def read(self, size=4096):
                return self.backend.read(size)
            def write(self, data):
                return self.backend.write(data)
            def close(self):
                self.backend.close()
        transport = MockSerial()  # no pyserial import or USB open
        client = ProtocolClient(transport)
        client.connect()
        self.assertTrue(client.hardware)
        client.hardware = False
        self.assertTrue(client.hardware)
        with self.assertRaises(ValueError):
            client.command(M.SET_SAFETY_CONFIG, config.safety.pack())
        client.config = config
        client.command(M.SET_SAFETY_CONFIG, config.safety.pack())
        client.command(M.SET_BASELINE, replace(config.baseline, version=10).pack())
        client.command(M.STAGE_CANDIDATE, replace(config.candidate, version=11).pack())
        self.assertEqual(transport.backend.device.candidate.version, 11)
        with self.assertRaises(ValueError):
            client.command(M.SET_BASELINE, replace(config.baseline, version=12, speed_p=101).pack())
        transport.close()

    def test_owner_budget_watchdog_survives_blocked_optimizer(self):
        config = fast_config("speed_pi")
        config.raw["gate_c"]["budget"]["max_total_ms"] = 3000
        device = SimulatedDevice()
        proposer = Proposer(config, delay=1.0)
        manager = self.manager(config, device)
        proposer.manager = manager
        result = manager.run_search(proposer)
        self.assertEqual(result["classification"], "BUDGET_EXHAUSTED", result)
        self.assertTrue(result["stop_confirmed"])
        self.assertEqual(len(proposer.registered), 1)
        self.assertEqual(device.batch_id, 0)
        self.assertIsNone(result.get("best_candidate"))

    def test_communication_loss_and_reboot_never_reconnect_or_retry(self):
        for injection in ("disconnect", "reboot"):
            device = SimulatedDevice()
            class BrokenTransport(SimulatedTransport):
                def __init__(self):
                    super().__init__(device)
                    self.triggered = False
                def read(self, size=4096):
                    if not self.triggered and self.device.state == S.RUN:
                        self.triggered = True
                        if injection == "disconnect":
                            raise OSError("injected RX disconnect")
                        self.device.reboot(2)
                    return super().read(size)
            result = self.manager(name=injection, transport_factory=BrokenTransport).run_single()
            self.assertEqual(result["classification"], "COMMUNICATION_INVALID", result)
            self.assertTrue(result["stop_confirmed"], result)
            tx = []
            for record in (self.root / injection / "raw.jsonl").read_text().splitlines():
                data = json.loads(record)
                if data["direction"] == "tx":
                    tx.extend(StreamDecoder().feed(bytes.fromhex(data["hex"])))
            self.assertEqual(sum(f.type == M.HELLO for f in tx), 1)
            self.assertEqual(sum(f.type == M.START_TRIAL for f in tx), 1)
            self.assertEqual(sum(f.type == M.CLEAR_FAULT for f in tx), 0)
            self.assertEqual(device.output, 0)

    def test_udp_failure_does_not_change_authoritative_records(self):
        config = fast_config()
        class DisconnectedVofa:
            def __init__(self, **options):
                self.dropped = 0
            def send(self, row):
                self.dropped += 1
            def close(self):
                pass
        with patch("gimbal_tuning.manager.VofaForwarder", DisconnectedVofa):
            result = self.manager(config, vofa={"port": 1347}).run_single()
        self.assertEqual(result["classification"], "COMPLETE", result)
        self.assertGreater(result["udp_dropped"], 0)
        offline = replay(self.root / "run")
        self.assertTrue(offline["trials"][0]["score"]["valid"])

    def test_hardware_conditional_diagnostic_without_acceptance_uses_bounded_timeout(self):
        raw = copy.deepcopy(fast_config().raw)
        raw["environment"] = "hardware"
        raw["baseline"]["algorithm"] = raw["candidate"]["algorithm"] = 1
        raw["confirmations"].update(conditional_trial_authorized=True, conditional_baseline_accepted=False)
        raw["gate_c"]["baseline_accepted"] = False
        config = Configuration(raw)
        with self.assertRaises(ValueError):
            config.validate("C", hardware=True)
        options = []
        class MockSerial(SerialTransport):
            def __init__(self, port, timeout):
                options.append((port, timeout))
                self.backend = SimulatedTransport()
                self.device = self.backend.device
            def read(self, size=4096):
                return self.backend.read(size)
            def write(self, data):
                return self.backend.write(data)
            def advance(self, milliseconds):
                self.backend.advance(milliseconds)
            def close(self):
                self.backend.close()
        with patch("gimbal_tuning.manager.SerialTransport", MockSerial):
            manager = ExperimentManager(config, self.root / "mock-hardware", hardware=True, port="MOCK", authorize_motion=True)
            result = manager.run_single(baseline=True)
        self.assertEqual(result["classification"], "COMPLETE", result)
        self.assertEqual(options[0][0], "MOCK")
        self.assertLessEqual(options[0][1], config.safety.heartbeat_timeout_ms / 8000)
        self.assertFalse(result["baseline_accepted"])
        self.assertFalse(config.raw["confirmations"]["conditional_baseline_accepted"])
        self.assertFalse(config.raw["gate_c"]["baseline_accepted"])
        self.assertFalse(result["physical_validation"])

    def test_manifest_edits_rejected_and_diagnostic_no_improvement_claim(self):
        config = fast_config(candidates=1)
        result = self.manager(config, name="legacy").run_search(Proposer(config))
        changed = copy.deepcopy(config.raw)
        changed["baseline"]["algorithm"] = changed["candidate"]["algorithm"] = 1
        changed["confirmations"]["conditional_baseline_accepted"] = True
        conditional = Configuration(changed)
        self.manager(conditional, name="conditional").run_search(Proposer(conditional))
        comparison = compare_baselines(self.root / "legacy", self.root / "conditional")
        self.assertTrue(comparison["comparable_configurations"])
        self.assertFalse(comparison["causal_improvement_established"])
        self.assertTrue(comparison["requires_independent_validation"])
        with (self.root / "legacy" / "raw.jsonl").open("ab") as handle:
            handle.write(b"\n")
        with self.assertRaisesRegex(ValueError, "integrity"):
            replay(self.root / "legacy")


if __name__ == "__main__":
    unittest.main()
