"""One bounded serial/RX/parser owner, independent of optimizer/scoring/storage threads."""
from collections import deque
from dataclasses import dataclass, field
import math
import queue
import secrets
import struct
import threading
import time
from .protocol import Frame, StreamDecoder, Message as M, Status, Telemetry, decode_result
from .transport import CommandRejected
from .scoring import telemetry_health


class ExperimentFailure(RuntimeError):
    def __init__(self, classification, message):
        self.classification = classification
        super().__init__(message)


@dataclass
class _Request:
    kind: int
    payload: bytes
    event: threading.Event = field(default_factory=threading.Event)
    result: object = None
    error: BaseException = None
    deadline: float = 0

    def finish(self, result=None, error=None):
        self.result, self.error = result, error
        self.event.set()


class IOOwner:
    """Only this thread reads, parses, allocates seq or writes the transport.

    Heartbeats continue while callers compute, wait for fsync, or perform BO.
    Serial read/write timeouts MUST be <= heartbeat_timeout/8 (manager sets them).
    A fatal error cancels pending/queued commands, sends STOP once, and awaits its
    zero ACK; no reconnect/retry/clear-fault exists. Device watchdog remains final
    authority if the OS, disk queue, transport or STOP confirmation fails.
    """
    def __init__(self, transport, safety, log, vofa=None, simulated_tick_s=0.0001, event_capacity=16384):
        self.transport, self.safety, self.log, self.vofa = transport, safety, log, vofa
        self.simulated = hasattr(transport, "advance") and hasattr(transport, "device")
        self.tick_sleep = simulated_tick_s if self.simulated else min(0.002, safety.heartbeat_timeout_ms / 16000)
        self.ack_timeout = max(0.1, min(2.0, safety.heartbeat_timeout_ms / 1000 * 2))
        self.epoch = time.monotonic()
        self.decoder = StreamDecoder()
        self.boot_id = self.session = self.seq = 0
        self.connected = False
        self.last_hb = None
        self.heartbeat_count = 0
        self.max_heartbeat_gap_ms = 0
        self.deadline_ms = None
        self.fatal = None
        self.requests = queue.Queue(32)
        self.pending = {}
        self.frames = deque(maxlen=event_capacity)
        self.frame_index = 0
        self.condition = threading.Condition()
        self.stop_requested = threading.Event()
        self.stop_done = threading.Event()
        self.stop_sent = False
        self.stop_confirmed = False
        self.stop_error = None
        self.stop_seq = None
        self.stop_expected_boot = None
        self.stop_deadline = 0
        self.halt = threading.Event()
        self.thread = threading.Thread(target=self._run, name="gimbal-serial-owner", daemon=True)
        self.thread.start()

    @property
    def now_ms(self):
        return self.transport.device.ms if self.simulated else (time.monotonic() - self.epoch) * 1000

    @property
    def parser_now_ms(self):
        return (time.monotonic() - self.epoch) * 1000

    def request(self, kind, payload=b""):
        self.check()
        request = _Request(kind, payload)
        try:
            self.requests.put_nowait(request)
        except queue.Full:
            raise ExperimentFailure("COMMUNICATION_INVALID", "bounded command queue exhausted") from None
        if not request.event.wait(self.ack_timeout * 2 + 0.1):
            self._fail(ExperimentFailure("COMMUNICATION_INVALID", "I/O owner command deadline exceeded"))
            raise self.fatal
        if request.error:
            raise request.error
        self.check()
        return request.result

    def check(self):
        if self.fatal:
            raise self.fatal
        if self.halt.is_set() or self.stop_requested.is_set():
            raise ExperimentFailure("COMMUNICATION_INVALID", "I/O owner is stopped")

    def cursor(self):
        with self.condition:
            return self.frame_index

    def since(self, cursor, wait=0):
        with self.condition:
            if wait and self.frame_index <= cursor and not self.fatal:
                self.condition.wait(wait)
            if self.frames and cursor < self.frames[0][0] - 1:
                raise ExperimentFailure("TELEMETRY_INVALID", "bounded telemetry history overflow")
            return [(i, f) for i, f in self.frames if i > cursor]

    def stop(self):
        self.stop_requested.set()
        if not self.stop_done.wait(self.ack_timeout + 0.2):
            self.stop_error = self.stop_error or "STOP owner/ACK deadline exceeded"
        return self.stop_confirmed

    def close(self):
        self.halt.set()
        self.thread.join(timeout=self.ack_timeout + 0.5)
        if self.thread.is_alive():
            raise OSError("I/O owner did not terminate; independent e-stop/watchdog required")

    def _fail(self, exc):
        if self.fatal is None:
            self.fatal = exc if isinstance(exc, ExperimentFailure) else ExperimentFailure("COMMUNICATION_INVALID", f"{type(exc).__name__}: {exc}")
            try:
                self.log.emit("events", dict(kind="fatal", classification=self.fatal.classification, message=str(self.fatal), clock_ms=self.now_ms))
            except BaseException:
                pass
            with self.condition:
                self.condition.notify_all()
        self.stop_requested.set()

    def _send(self, frame, emergency=False):
        raw = frame.encode()
        try:
            self.log.emit("raw", dict(direction="tx", clock_ms=self.parser_now_ms, monotonic_ns=time.monotonic_ns(), hex=raw.hex()))
        except BaseException:
            if not emergency:
                raise
        count = self.transport.write(raw)
        if count != len(raw):
            raise OSError("partial serial write; command uncertain, no retry")

    def _begin_stop(self):
        self.stop_sent = True
        self.connected = False
        self.deadline_ms = None
        error = self.fatal or ExperimentFailure("STOPPED", "explicit STOP cancels authorization")
        for request in self.pending.values():
            request.finish(error=error)
        self.pending.clear()
        while True:
            try:
                self.requests.get_nowait().finish(error=error)
            except queue.Empty:
                break
        # STOP is outside the ordinary seq domain. A fresh echo nonce prevents an
        # old session's delayed STOP ACK from confirming this new stop attempt.
        self.stop_seq = secrets.randbits(32) or 1
        self.stop_expected_boot = self.stop_expected_boot or self.boot_id or None
        self.stop_deadline = time.monotonic() + self.ack_timeout
        try:
            self._send(Frame(M.STOP, 0, 0, self.stop_seq), emergency=True)
        except BaseException as exc:
            self.stop_error = f"STOP write unconfirmed: {exc}"
        # Remain the sole RX owner until ACK or deadline, even after a write error.

    def _dispatch(self, request):
        if request.kind == M.HELLO:
            if self.connected or self.seq:
                raise ExperimentFailure("COMMUNICATION_INVALID", "HELLO is explicit initial connection only; no reconnect")
            seq = 0
            frame = Frame(M.HELLO)
        else:
            if not self.connected or self.seq >= 0xffffffff:
                raise ExperimentFailure("COMMUNICATION_INVALID", "no live session or sequence exhausted")
            self.seq += 1
            seq = self.seq
            frame = Frame(request.kind, self.boot_id, self.session, seq, request.payload)
        request.deadline = time.monotonic() + self.ack_timeout
        self.pending[seq] = request
        self._send(frame)
        if request.kind == M.HEARTBEAT:
            now = self.now_ms
            if self.last_hb is not None:
                self.max_heartbeat_gap_ms = max(self.max_heartbeat_gap_ms, now - self.last_hb)
            self.last_hb = now
            self.heartbeat_count += 1

    def _receive(self, frame):
        # Record all valid wire evidence before semantic checks, including malformed payloads.
        with self.condition:
            self.frame_index += 1
            index = self.frame_index
            self.frames.append((index, frame))
            self.condition.notify_all()
        try:
            self.log.emit("frames", dict(index=index, type=frame.type, boot_id=frame.boot_id,
                                         session=frame.session, seq=frame.seq, payload_hex=frame.payload.hex()))
        except BaseException:
            if not self.stop_sent:
                raise
        if self.stop_sent:
            if frame.type == M.ACK and frame.seq == self.stop_seq:
                if (len(frame.payload) != 56 or struct.unpack("<II", frame.payload[:8]) != (M.STOP, 0)
                        or frame.session != 0 or not frame.boot_id
                        or (self.stop_expected_boot is not None and frame.boot_id != self.stop_expected_boot)):
                    raise ValueError("malformed STOP confirmation")
                status = Status.unpack(frame.payload[8:])
                if status.flags & (8 | 16) or status.state not in (7, 8):
                    raise ValueError("STOP ACK did not confirm zero and revoked batch")
                self.stop_confirmed = True
                self.stop_done.set()
            return
        if self.connected and (frame.boot_id != self.boot_id or frame.session != self.session):
            if frame.boot_id:
                self.stop_expected_boot = frame.boot_id
            raise ExperimentFailure("COMMUNICATION_INVALID", "boot/session changed; authorization lost")
        if frame.type == M.TELEMETRY:
            try:
                row = Telemetry.unpack(frame.payload)
                self.log.emit("telemetry", dict(index=index, boot_id=frame.boot_id, session=frame.session, row=row))
                values = list(row.__dict__.values())[15:]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError("nonfinite telemetry")
            except (ValueError, OverflowError) as exc:
                self.log.emit("events", dict(kind="invalid_telemetry", index=index, reason=str(exc)))
                raise ExperimentFailure("TELEMETRY_INVALID", str(exc)) from exc
            if row.fault:
                raise ExperimentFailure("PROTECTION", f"telemetry fault {row.fault}")
            try:
                telemetry_health(row, self.safety)
            except ValueError as exc:
                raise ExperimentFailure("PROTECTION", str(exc)) from exc
            if self.vofa:
                self.vofa.send(row)
        elif frame.type == M.FAULT:
            status = Status.unpack(frame.payload)
            raise ExperimentFailure("PROTECTION", f"device fault {status.fault}")
        elif frame.type == M.RESULT:
            result, status = decode_result(frame.payload)
            if result.classification != 0 or result.fault or status.fault:
                classification = "PREPARATION_FAILED" if result.classification == 3 else "PROTECTION" if result.classification == 2 else "STOPPED"
                raise ExperimentFailure(classification, f"device RESULT class={result.classification}, fault={result.fault}")
        if frame.seq not in self.pending:
            return
        request = self.pending[frame.seq]
        if frame.type == M.NACK:
            if len(frame.payload) != 56:
                raise ValueError("malformed NACK")
            kind, error = struct.unpack("<II", frame.payload[:8])
            request.finish(error=CommandRejected(kind, error, Status.unpack(frame.payload[8:])))
            del self.pending[frame.seq]
            if request.kind == M.HEARTBEAT:
                raise ExperimentFailure("COMMUNICATION_INVALID", "heartbeat rejected")
            return
        expected = {M.HELLO: M.INFO, M.GET_STATUS: M.STATUS, M.GET_CONFIG: M.CONFIG}.get(request.kind, M.ACK)
        if frame.type != expected:
            return
        size = {M.INFO: 80, M.STATUS: 48, M.CONFIG: 172, M.ACK: 56}[expected]
        if len(frame.payload) != size:
            raise ValueError("response payload length mismatch")
        if expected == M.INFO:
            if not frame.boot_id or not frame.session or struct.unpack("<8I", frame.payload[:32]) != (1, 65536, 3, 0x205, 0x206, 500000, 1000, 124):
                raise ValueError("device identity mismatch")
            self.boot_id, self.session = frame.boot_id, frame.session
            self.connected = True
        if expected == M.ACK and struct.unpack("<II", frame.payload[:8]) != (request.kind, 0):
            raise ValueError("ACK command/error mismatch")
        request.finish(frame)
        del self.pending[frame.seq]

    def _run(self):
        try:
            while not self.halt.is_set():
                try:
                    if self.stop_requested.is_set() and not self.stop_sent:
                        self._begin_stop()
                    if self.stop_sent:
                        if time.monotonic() >= self.stop_deadline and not self.stop_done.is_set():
                            self.stop_error = self.stop_error or "STOP ACK missing; zero unconfirmed; use independent e-stop"
                            self.stop_done.set()
                    else:
                        if self.log.error:
                            raise OSError("asynchronous log writer failed: " + self.log.error)
                        if self.deadline_ms is not None and self.now_ms >= self.deadline_ms:
                            raise ExperimentFailure("BUDGET_EXHAUSTED", "host batch wall-time budget exhausted")
                        if self.connected:
                            gap = self.now_ms - self.last_hb if self.last_hb is not None else 0
                            if self.last_hb is not None and gap >= self.safety.heartbeat_timeout_ms * 0.8:
                                raise ExperimentFailure("COMMUNICATION_INVALID", "host heartbeat scheduling deadline missed")
                            if self.last_hb is None or gap >= max(1, self.safety.heartbeat_timeout_ms // 4):
                                self._dispatch(_Request(M.HEARTBEAT, b""))
                        try:
                            request = self.requests.get_nowait()
                        except queue.Empty:
                            request = None
                        if request:
                            self._dispatch(request)
                        if any(time.monotonic() >= request.deadline for request in self.pending.values()):
                            raise ExperimentFailure("COMMUNICATION_INVALID", "command ACK timeout; no retry")
                    raw = self.transport.read(4096)
                    if raw:
                        try:
                            self.log.emit("raw", dict(direction="rx", clock_ms=self.parser_now_ms, monotonic_ns=time.monotonic_ns(), hex=raw.hex()))
                        except BaseException:
                            if not self.stop_sent:
                                raise
                    rejected = self.decoder.rejected
                    frames = self.decoder.feed(raw, self.parser_now_ms)
                    if self.decoder.rejected != rejected:
                        raise ExperimentFailure("COMMUNICATION_INVALID", "CRC/version/length/truncated-frame rejection")
                    for frame in frames:
                        self._receive(frame)
                    if self.simulated:
                        self.transport.advance(1)
                except BaseException as exc:
                    self._fail(exc)
                time.sleep(self.tick_sleep)
        finally:
            error = self.fatal or ExperimentFailure("COMMUNICATION_INVALID", "I/O owner closed")
            for request in self.pending.values():
                request.finish(error=error)
            while True:
                try:
                    self.requests.get_nowait().finish(error=error)
                except queue.Empty:
                    break
            self.transport.close()
