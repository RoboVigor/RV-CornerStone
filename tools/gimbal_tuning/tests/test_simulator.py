from dataclasses import replace
from pathlib import Path
import struct
import unittest
from gimbal_tuning.cli import run_simulation
from gimbal_tuning.config import Configuration
from gimbal_tuning.protocol import (Frame, StreamDecoder, Message as M, State as S, Fault as F,
                                    Error as E, Classification as C, Telemetry, Status, decode_result)
from gimbal_tuning.simulator import SimulatedDevice, SimulatedTransport
from gimbal_tuning.transport import ProtocolClient, ProtocolError, CommandRejected

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


class DeviceTests(unittest.TestCase):
    def setUp(self):
        self.config = Configuration.load(EXAMPLES / "simulation.json")
        self.device = SimulatedDevice(123)
        self.transport = SimulatedTransport(self.device)
        self.client = ProtocolClient(self.transport)
        self.client.connect()
        self.seq = 0

    def wire(self, frame):
        self.device.receive(frame.encode())
        return StreamDecoder().feed(self.device.drain(), self.device.ms)

    def command(self, kind, payload=b"", seq=None):
        self.seq = self.seq + 1 if seq is None else seq
        frame = Frame(kind, self.device.boot_id, self.device.session, self.seq, payload)
        return frame, self.wire(frame)

    def setup_batch(self, safety=None, algorithm=0):
        for kind, payload in [
            (M.SET_SAFETY_CONFIG, (safety or self.config.safety).pack()),
            (M.SET_BASELINE, replace(self.config.baseline, algorithm=algorithm).pack()),
            (M.STAGE_CANDIDATE, replace(self.config.candidate, algorithm=algorithm).pack()),
            (M.HEARTBEAT, b""), (M.START_BATCH, struct.pack("<II", 42, 2)),
        ]:
            _, responses = self.command(kind, payload)
            self.assertEqual(responses[-1].type, M.ACK)

    def start(self, trial=None):
        return self.command(M.START_TRIAL, (trial or self.config.trial).pack())

    def test_default_zero_queries_and_lengths(self):
        self.assertFalse(self.device.motion)
        self.assertEqual(self.device.output, 0)
        for kind, response, size in [(M.GET_STATUS, M.STATUS, 48), (M.GET_CONFIG, M.CONFIG, 172)]:
            _, replies = self.command(kind)
            self.assertEqual((replies[-1].type, len(replies[-1].payload)), (response, size))
        _, replies = self.start()
        self.assertEqual(replies[-1].type, M.NACK)
        self.assertEqual(self.device.output, 0)

    def test_duplicate_start_and_replay_order(self):
        self.setup_batch()
        frame, responses = self.start()
        self.assertEqual(responses[-1].type, M.ACK)
        self.assertEqual(self.device.trials_started, 1)
        self.assertEqual(self.wire(frame)[-1].type, M.ACK)
        self.assertEqual(self.device.trials_started, 1)
        conflict = replace(frame, payload=replace(self.config.trial, target=-0.1).pack())
        reply = self.wire(conflict)[-1]
        self.assertEqual(struct.unpack("<II", reply.payload[:8])[1], E.REPLAY)
        _, replies = self.command(M.GET_STATUS)
        self.assertEqual(self.wire(frame)[-1].type, M.NACK)
        older = Frame(M.HEARTBEAT, 123, self.device.session, 1)
        self.assertEqual(self.wire(older)[-1].type, M.NACK)

    def test_rejected_attempt_consumes_sequence(self):
        frame, replies = self.command(M.START_TRIAL, b"bad")
        self.assertEqual(replies[-1].type, M.NACK)
        self.assertEqual(self.device.last_command_seq, frame.seq)
        self.assertEqual(self.wire(frame)[-1].type, M.NACK)
        corrected = replace(frame, type=M.GET_STATUS, payload=b"")
        self.assertEqual(struct.unpack("<II", self.wire(corrected)[-1].payload[:8])[1], E.REPLAY)

    def test_duplicate_heartbeat_not_fresh_and_fault_latched(self):
        self.setup_batch()
        heartbeat, _ = self.command(M.HEARTBEAT)
        self.device.advance(80)
        self.wire(heartbeat)
        self.assertEqual(self.device.heartbeat_ms, 0)
        self.device.advance(21)
        self.assertEqual(self.device.fault, F.HEARTBEAT)
        self.assertEqual(self.device.state, S.FAULT_LATCHED_ZERO)
        self.assertEqual(self.device.output, 0)
        self.wire(Frame(M.HELLO))
        self.assertEqual(self.device.fault, F.HEARTBEAT)
        self.seq = 0
        self.command(M.HEARTBEAT)
        self.command(M.CLEAR_FAULT)
        self.assertEqual(self.device.state, S.IDLE_ZERO)
        self.assertFalse(self.device.motion)
        self.assertEqual(self.device.batch_id, 0)

    def test_stop_is_global_barrier_and_does_not_clear_fault(self):
        self.setup_batch()
        start, _ = self.start()
        self.device.advance(1)
        stop_replies = self.wire(Frame(M.STOP, 999, 999, 0))
        self.assertEqual(stop_replies[-1].session, 0)
        self.assertEqual(self.device.state, S.STOPPED_ZERO)
        self.assertEqual(self.device.output, 0)
        self.assertEqual(self.wire(start)[-1].type, M.NACK)
        self.assertEqual(self.device.session, 0)
        self.wire(Frame(M.HELLO))
        self.assertNotEqual(self.device.session, start.session)
        self.assertEqual(self.wire(start)[-1].type, M.NACK)
        self.device.fault = F.FEEDBACK
        self.device.state = S.FAULT_LATCHED_ZERO
        self.wire(Frame(M.STOP))
        self.assertEqual(self.device.fault, F.FEEDBACK)
        self.assertEqual(self.device.state, S.FAULT_LATCHED_ZERO)

    def test_hello_revokes_motion_and_boot_isolation(self):
        self.setup_batch()
        start, _ = self.start()
        self.device.advance(1)
        old_session = self.device.session
        replies = self.wire(Frame(M.HELLO))
        self.assertEqual(replies[-1].type, M.INFO)
        self.assertGreater(self.device.session, old_session)
        self.assertEqual(self.device.batch_id, 0)
        self.assertEqual(self.device.output, 0)
        self.device.reboot(124)
        self.wire(Frame(M.HELLO))
        self.assertEqual(self.wire(start)[-1].type, M.NACK)
        self.assertFalse(self.device.motion)
        with self.assertRaises(ValueError):
            self.device.reboot(124)
        with self.assertRaises(ValueError):
            SimulatedDevice(0)

    def test_corrupt_truncated_versioned_start_cannot_move(self):
        self.setup_batch()
        start = Frame(M.START_TRIAL, 123, self.device.session, self.seq + 1, self.config.trial.pack()).encode()
        bad = bytearray(start)
        bad[-1] ^= 1
        unknown = bytearray(start)
        unknown[2] = 99
        self.device.receive(bad + unknown)
        self.assertFalse(self.device.motion)
        self.device.receive(start[:-2])
        self.device.advance(101)
        self.assertFalse(self.device.motion)
        self.assertGreater(self.device.decoder.rejected, 0)
        self.assertEqual(self.device.output, 0)

    def test_parameter_staging_atomic_version_limits_and_session_barrier(self):
        self.setup_batch()
        self.assertIsNone(self.device.active)
        _, replies = self.command(M.STAGE_CANDIDATE, replace(self.config.candidate, version=2).pack())
        self.assertEqual(replies[-1].type, M.NACK)
        _, replies = self.command(M.STAGE_CANDIDATE, replace(self.config.candidate, version=3, algorithm=1).pack())
        self.assertEqual(replies[-1].type, M.NACK)
        self.wire(Frame(M.HELLO))
        self.seq = 0
        self.assertEqual(self.device.max_version, 1)
        _, replies = self.command(M.STAGE_CANDIDATE, self.config.candidate.pack())
        self.assertEqual(replies[-1].type, M.ACK)
        self.assertEqual(self.device.candidate.version, 2)
        self.assertIsNone(self.device.active)

    def test_safety_frozen_during_batch_and_bad_payloads(self):
        self.setup_batch()
        _, replies = self.command(M.SET_SAFETY_CONFIG, self.config.safety.pack())
        self.assertEqual(struct.unpack("<II", replies[-1].payload[:8])[1], E.BAD_STATE)
        for kind, payload in [(M.STOP, b"bad"), (M.GET_STATUS, b"bad"), (M.START_TRIAL, b"bad"), (200, b"")]:
            _, replies = self.command(kind, payload)
            self.assertEqual(replies[-1].type, M.NACK)
        self.assertEqual(self.device.batch_id, 42)

    def test_protections_zero_cancel_no_fallback(self):
        for injection, fault in [
            (lambda d: setattr(d, "pos", 0.95), F.POSITION),
            (lambda d: setattr(d, "vel", 3), F.SPEED),
            (lambda d: setattr(d, "pair_offset", 0.06), F.PAIR_MISMATCH),
            (lambda d: setattr(d, "currents", [0, 21]), F.CURRENT),
            (lambda d: setattr(d, "temperatures", [25, 71]), F.TEMPERATURE),
            (lambda d: setattr(d, "other_targets", [0, 1, 0]), F.OTHER_JOINT),
            (lambda d: setattr(d, "pos", float("nan")), F.NONFINITE),
        ]:
            with self.subTest(fault=fault):
                self.setUp()
                self.setup_batch()
                self.start()
                injection(self.device)
                self.device.advance(1)
                self.assertEqual(self.device.fault, fault)
                self.assertEqual(self.device.output, 0)
                self.assertEqual(self.device.batch_id, 0)
                self.assertEqual(self.device.state, S.FAULT_LATCHED_ZERO)

    def test_either_motor_feedback_loss(self):
        for motor in range(2):
            self.setUp()
            self.setup_batch()
            self.start()
            self.device.feedback_enabled[motor] = False
            self.device.advance(51)
            self.assertEqual(self.device.fault, F.FEEDBACK)
            self.assertEqual(self.device.output, 0)
            _, replies = self.command(M.CLEAR_FAULT)
            self.assertEqual(replies[-1].type, M.NACK)

    def test_complete_position_speed_fixed_nonzero_origin(self):
        for mode, origin in [(0, 0), (1, 0), (0, -0.1)]:
            with self.subTest(mode=mode, origin=origin):
                raw = dict(self.config.raw)
                raw["trial"] = dict(raw["trial"], mode=mode, origin=origin)
                report = run_simulation(Configuration(raw))
                result, rows = report["result"], report["run"]
                self.assertEqual(result.classification, C.COMPLETE)
                self.assertEqual(rows[0].sample_seq, result.run_start_seq)
                self.assertEqual(rows[-1].sample_seq, result.run_end_seq)
                self.assertEqual(rows[-1].tick_us - rows[0].tick_us, 1000000)
                self.assertEqual(rows[-1].output, 0)
                self.assertEqual(rows[-1].command0, 0)
                self.assertEqual(report["telemetry"][-1].state, S.COMPLETE_ZERO)
                self.assertEqual(report["telemetry"][-1].sample_seq, (result.run_end_seq + 1) & 0xffffffff)
                self.assertEqual(report["status"].flags & (8 | 16), 0)
                self.assertEqual({r.active_version for r in rows}, {2})
                if mode == 1:
                    self.assertEqual(rows[0].target_vel, 0)
                    self.assertTrue(all(0 <= r.target_vel <= 0.100001 for r in rows))
                    self.assertAlmostEqual(rows[-1].target_vel, 0.1, places=6)
                else:
                    self.assertTrue(all(abs(r.target_pos - 0.1) < 1e-6 for r in rows))

    def test_lost_rest_resets_hold_and_apply_uses_same_confirmed_sample(self):
        self.setup_batch(replace(self.config.safety, settle_hold_ms=10))
        self.start()
        while self.device.state != S.SETTLE:
            self.device.advance(1)
        self.device.advance(3)
        self.assertIsNotNone(self.device.settle_since)
        self.device.vel = 0.03  # above approved settle speed, below protection speed
        self.device.advance(1)
        self.assertIsNone(self.device.settle_since)
        self.assertEqual(self.device.active.version, 1)
        self.device.vel = 0
        self.device.advance(1)
        confirmed_hold_start = self.device.ms
        self.device.advance(9)
        self.assertEqual(self.device.state, S.SETTLE)
        self.device.advance(1)
        self.assertEqual(self.device.state, S.RUN)
        self.assertEqual(self.device.ms, confirmed_hold_start + 10)
        self.assertEqual(self.device.applied_seq, self.device.sample_seq)
        self.assertEqual(self.device.run_start_seq, self.device.applied_seq)
        self.assertEqual(self.device.active.version, 2)

    def test_conditional_integration_is_separate_algorithm(self):
        observed = []
        for algorithm in (0, 1):
            self.setUp()
            self.setup_batch(replace(self.config.safety, max_output=50, max_integral=50), algorithm)
            self.start(replace(self.config.trial, target=0.8))
            for _ in range(150):
                if self.device.ms % 30 == 0:
                    self.command(M.HEARTBEAT)
                self.device.advance(1)
            self.assertEqual(self.device.state, S.RUN)
            observed.append(self.device.integral)
        self.assertGreater(observed[0], 0)
        self.assertEqual(observed[1], 0)

    def test_prepare_settle_and_batch_deadlines(self):
        cases = [
            (replace(self.config.safety, prepare_timeout_ms=1), replace(self.config.trial, origin=-0.1), F.PREPARE_TIMEOUT),
            (replace(self.config.safety, settle_timeout_ms=1, settle_hold_ms=1), replace(self.config.trial, origin=-0.1), F.SETTLE_TIMEOUT),
            (replace(self.config.safety, batch_timeout_ms=80, max_trial_ms=75), replace(self.config.trial, duration_ms=75), F.BATCH_TIMEOUT),
        ]
        for safety, trial, fault in cases:
            with self.subTest(fault=fault):
                self.setUp()
                self.setup_batch(safety)
                self.start(trial)
                for _ in range(15000):
                    if self.device.ms % 30 == 0:
                        self.command(M.HEARTBEAT)
                    self.device.advance(1)
                    if self.device.fault:
                        break
                self.assertEqual(self.device.fault, fault)
                self.assertEqual(self.device.state, S.FAULT_LATCHED_ZERO)
                self.assertEqual(self.device.output, 0)
                self.assertEqual(self.device.batch_id, 0)

    def test_next_trial_requires_staging_and_budget(self):
        self.setup_batch()
        self.start()
        for _ in range(1200):
            if self.device.ms % 30 == 0:
                self.command(M.HEARTBEAT)
            self.device.advance(1)
            if self.device.state == S.COMPLETE_ZERO:
                break
        self.assertEqual(self.device.state, S.COMPLETE_ZERO)
        self.assertIsNone(self.device.candidate)
        _, replies = self.start(replace(self.config.trial, trial_id=2))
        self.assertEqual(replies[-1].type, M.NACK)
        self.command(M.STAGE_CANDIDATE, replace(self.config.candidate, version=3).pack())
        _, replies = self.start(replace(self.config.trial, trial_id=2))
        self.assertEqual(struct.unpack("<II", replies[-1].payload[:8])[1], E.NOT_READY)
        self.device.advance(1)  # required zero endpoint cannot be superseded by a new START
        _, replies = self.start(replace(self.config.trial, trial_id=2))
        self.assertEqual(replies[-1].type, M.ACK)
        for _ in range(15000):
            if self.device.ms % 30 == 0:
                self.command(M.HEARTBEAT)
            self.device.advance(1)
            if self.device.state in (S.COMPLETE_ZERO, S.FAULT_LATCHED_ZERO):
                break
        self.assertEqual(self.device.state, S.COMPLETE_ZERO)
        self.device.advance(1)
        self.command(M.STAGE_CANDIDATE, replace(self.config.candidate, version=4).pack())
        _, replies = self.start(replace(self.config.trial, trial_id=3))
        self.assertEqual(struct.unpack("<II", replies[-1].payload[:8])[1], E.BUDGET)

    def test_uncertain_start_is_never_retried(self):
        class SilentTransport:
            def __init__(self):
                self.writes = []
            def read(self, size):
                return b""
            def write(self, data):
                self.writes.append(data)
                return len(data)
        transport = SilentTransport()
        client = ProtocolClient(transport, timeout=0.001)
        client.boot_id = client.session = 1
        with self.assertRaises(TimeoutError):
            client.command(M.START_TRIAL, self.config.trial.pack())
        self.assertEqual(len(transport.writes), 1)

    def test_transport_client_no_auto_retry_and_hardware_gate(self):
        self.client.command(M.GET_STATUS)
        self.client.stop()
        with self.assertRaises(ProtocolError):
            self.client.command(M.START_BATCH, struct.pack("<II", 1, 1))
        self.client.connect()
        self.client.hardware = True
        with self.assertRaises(ValueError):
            self.client.command(M.SET_SAFETY_CONFIG, self.config.safety.pack())
        self.client.config = self.config
        with self.assertRaises(ValueError):
            self.client.command(M.SET_SAFETY_CONFIG, self.config.safety.pack())
        self.client.command(M.GET_STATUS)  # Gate A still works, without motion
        self.client.hardware = False
        self.device.reboot(124)
        with self.assertRaises(ProtocolError):
            self.client.command(M.GET_STATUS)
        self.client.connect()
        self.assertEqual(self.client.boot_id, 124)
        self.assertFalse(self.device.motion)


if __name__ == "__main__":
    unittest.main()
