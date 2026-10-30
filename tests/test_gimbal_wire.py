"""Cross-test Python's wire codec against the *actual* C firmware core, no MCU.

Run from repo root: PYTHONPATH=tools/gimbal_tuning python -m unittest discover -s tests -v
The C shim supplies stationary, fresh fake feedback only; it does not validate a plant.
"""
import ctypes
from dataclasses import replace
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest

from gimbal_tuning.protocol import (Frame, Message as M, StreamDecoder, Status,
                                   Telemetry, Trial, State, Parameters, decode_result)
from gimbal_tuning.config import Configuration
from gimbal_tuning.scoring import evaluate_trial
from gimbal_tuning.manager import ExperimentManager

ROOT = Path(__file__).resolve().parents[1]


class FirmwareWireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc = shutil.which("cc")
        if not cc:
            raise unittest.SkipTest("native C compiler not installed")
        cls.temp = tempfile.TemporaryDirectory(prefix="gimbal-wire-")
        library = Path(cls.temp.name) / "core.so"
        subprocess.run([cc, "-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic",
                        "-shared", "-fPIC", "-I" + str(ROOT / "driver/inc"),
                        str(ROOT / "driver/src/Driver_GimbalTuning.c"),
                        str(ROOT / "tests/gimbal_wire_shim.c"), "-lm", "-o", str(library)], check=True)
        cls.lib = ctypes.CDLL(str(library))
        cls.lib.gts_reset.argtypes = [ctypes.c_uint32]
        cls.lib.gts_tick.argtypes = [ctypes.c_uint32]
        cls.lib.gts_offline.argtypes = [ctypes.c_uint32]
        cls.lib.gts_feed.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32]
        cls.lib.gts_read.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        cls.lib.gts_read.restype = ctypes.c_uint32

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def setUp(self):
        self.config = Configuration.load(ROOT / "tools/gimbal_tuning/examples/simulation.json")
        self.now, self.seq, self.boot, self.session = 1000, 0, 0, 0
        self.decoder = StreamDecoder()
        self.lib.gts_reset(0x10203040)
        self.frames = []
        info = self.send(Frame(M.HELLO))[-1]
        self.boot, self.session = info.boot_id, info.session
        self.assertEqual(info.type, M.INFO)
        self.assertEqual(struct.unpack("<8I", info.payload[:32]), (1, 65536, 3, 517, 518, 500000, 1000, 124))
        self.assertEqual(Status.unpack(info.payload[32:]).flags, 0)

    def drain(self):
        buf = ctypes.create_string_buffer(65536)
        count = self.lib.gts_read(buf, len(buf))
        frames = self.decoder.feed(buf.raw[:count], self.now / 1000)
        self.frames.extend(frames)
        return frames

    def send(self, frame):
        data = frame.encode()
        self.lib.gts_feed(data, len(data), self.now)
        return self.drain()

    def command(self, kind, payload=b""):
        self.seq += 1
        result = self.send(Frame(kind, self.boot, self.session, self.seq, payload))[-1]
        self.assertNotEqual(result.type, M.NACK, result.payload.hex())
        return result

    def configure(self):
        self.command(M.SET_SAFETY_CONFIG, self.config.safety.pack())
        self.command(M.SET_BASELINE, self.config.baseline.pack())
        self.command(M.STAGE_CANDIDATE, self.config.candidate.pack())
        self.command(M.HEARTBEAT)

    def tick(self, count=1):
        frames = []
        for _ in range(count):
            self.now += 1000
            self.lib.gts_tick(self.now)
            frames.extend(self.drain())
        return frames

    def test_config_bytes_and_no_motion_from_queries(self):
        self.configure()
        config = self.command(M.GET_CONFIG).payload
        self.assertEqual(len(config), 172)
        self.assertEqual(config[:4], struct.pack("<I", 1))
        self.assertEqual(config[4:144], self.config.safety.pack())
        self.assertEqual(config[144:148], struct.pack("<I", 1))
        self.assertEqual(config[148:], self.config.baseline.pack())
        status = Status.unpack(self.command(M.GET_STATUS).payload)
        self.assertEqual(status.active_version, 0)  # staging isn't application
        self.assertFalse(status.flags & 24)

    def test_complete_position_and_speed_windows(self):
        for mode in (0, 1):
            with self.subTest(mode=mode):
                self.setUp()
                self.configure()
                self.command(M.START_BATCH, struct.pack("<II", 7, 1))
                trial = replace(self.config.trial, mode=mode)
                window_start = len(self.frames)
                self.command(M.START_TRIAL, trial.pack())
                result = None
                for i in range(2000):
                    if i % 25 == 0:
                        self.command(M.HEARTBEAT)
                    for frame in self.tick():
                        if frame.type == M.RESULT:
                            result, status = decode_result(frame.payload)
                    if result:
                        break
                self.assertIsNotNone(result)
                self.assertEqual(result.classification, 0)
                self.assertEqual(status.state, State.COMPLETE_ZERO)
                self.assertFalse(status.flags & 16)
                rows = [Telemetry.unpack(f.payload) for f in self.frames if f.type == M.TELEMETRY]
                run = [r for r in rows if r.state == State.RUN]
                self.assertEqual((run[0].sample_seq, run[-1].sample_seq), (result.run_start_seq, result.run_end_seq))
                self.assertEqual((run[0].tick_us, run[-1].tick_us), (result.run_start_us, result.run_end_us))
                self.assertTrue(all(r.active_version == 2 and r.applied_seq == result.run_start_seq for r in run))
                self.assertEqual((run[-1].command0, run[-1].command1, run[-1].output), (0, 0, 0))
                after = [Telemetry.unpack(f.payload) for f in self.tick() if f.type == M.TELEMETRY]
                self.assertEqual(after[0].state, State.COMPLETE_ZERO)
                self.assertEqual((after[0].command0, after[0].command1), (0, 0))
                self.command(M.GET_STATUS)
                expected = dict(boot_id=self.boot, session=self.session, trial=dict(trial.__dict__), version=2, batch_id=7)
                score = evaluate_trial(self.frames[window_start:], expected, self.config.safety)
                self.assertTrue(score.valid, score.reasons)
                self.command(M.END_BATCH)
                end = Status.unpack(self.command(M.GET_STATUS).payload)
                self.assertEqual(end.state, State.IDLE_ZERO)
                self.assertFalse(end.flags & 24)
                self.assertEqual(end.batch_id, 0)

    def test_real_manager_against_same_firmware_across_explicit_sessions(self):
        lib = self.lib

        class FirmwareTransport:
            is_hardware = False

            def __init__(self):
                self.ms = 0
                self.device = self  # IOOwner's explicit virtual-clock transport interface
                self.closed = False
                lib.gts_reset(0x10203040)

            def write(self, data):
                lib.gts_feed(data, len(data), (self.ms + 1) * 1000)
                return len(data)

            def read(self, size=4096):
                buf = ctypes.create_string_buffer(size)
                n = lib.gts_read(buf, size)
                return buf.raw[:n]

            def advance(self, count):
                for _ in range(count):
                    self.ms += 1
                    lib.gts_tick((self.ms + 1) * 1000)

            def close(self):
                self.closed = True  # MCU stays alive, as on a real explicit port reopen.

        transport = FirmwareTransport()

        def reopen():
            transport.closed = False
            return transport

        with tempfile.TemporaryDirectory(prefix="gimbal-firmware-manager-") as directory:
            summaries = []
            for run in (1, 2):
                manager = ExperimentManager(self.config, Path(directory) / str(run), transport_factory=reopen)
                summary = manager.run_single(baseline=True)
                self.assertEqual(summary["classification"], "COMPLETE", summary.get("error"))
                self.assertTrue(summary["zero_confirmed"])
                self.assertTrue(transport.closed)
                summaries.append(summary)
            a, b = (s["trials"][0] for s in summaries)
            self.assertNotEqual(a["expected"]["session"], b["expected"]["session"])
            self.assertGreater(b["parameters"]["version"], a["parameters"]["version"])

    def test_stop_barrier_duplicate_and_fault(self):
        self.configure()
        self.command(M.START_BATCH, struct.pack("<II", 7, 2))
        self.seq += 1
        start = Frame(M.START_TRIAL, self.boot, self.session, self.seq, self.config.trial.pack())
        self.assertEqual(self.send(start)[-1].type, M.ACK)
        self.tick()
        self.assertEqual(self.send(start)[-1].type, M.ACK)
        self.lib.gts_offline(1)
        fault = self.tick()
        self.assertTrue(any(f.type == M.FAULT for f in fault))
        status = Status.unpack(self.command(M.GET_STATUS).payload)
        self.assertEqual((status.state, status.fault), (State.FAULT_LATCHED_ZERO, 2))
        self.assertFalse(status.flags & 24)
        stopped = self.send(Frame(M.STOP))[-1]
        self.assertEqual(stopped.session, 0)
        self.assertEqual(Status.unpack(stopped.payload[8:]).fault, 2)
        self.assertEqual(self.send(start)[-1].type, M.NACK)
        old = self.session
        hello = self.send(Frame(M.HELLO))[-1]
        self.assertGreater(hello.session, old)
        self.assertEqual(Status.unpack(hello.payload[32:]).fault, 2)

    def test_bad_crc_and_rejected_start_cannot_later_run(self):
        self.configure()
        self.seq += 1
        rejected = Frame(M.START_TRIAL, self.boot, self.session, self.seq, self.config.trial.pack())
        self.assertEqual(self.send(rejected)[-1].type, M.NACK)
        self.command(M.START_BATCH, struct.pack("<II", 7, 1))
        self.assertEqual(self.send(rejected)[-1].type, M.NACK)
        self.seq += 1
        wire = bytearray(Frame(M.START_TRIAL, self.boot, self.session, self.seq, self.config.trial.pack()).encode())
        wire[-1] ^= 1
        self.lib.gts_feed(bytes(wire), len(wire), self.now)
        self.tick(2)
        status = Status.unpack(self.command(M.GET_STATUS).payload)
        self.assertFalse(status.flags & 16)
        self.assertGreater(status.rx_dropped, 0)


if __name__ == "__main__":
    unittest.main()
