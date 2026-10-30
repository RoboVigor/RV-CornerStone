"""Wire contract v1. Explicit little-endian fields; never native C layouts."""
from dataclasses import dataclass, fields
from enum import IntEnum
import math
import struct

VERSION = 1
SYNC = b"\xa5\x5a"
MAX_PAYLOAD = 256
HEADER = struct.Struct("<2sBBHIII")
MAX_FRAME = HEADER.size + MAX_PAYLOAD + 2
FRAME_TIMEOUT_MS = 100


class Message(IntEnum):
    HELLO = 1
    GET_STATUS = 2
    GET_CONFIG = 3
    SET_SAFETY_CONFIG = 10
    SET_BASELINE = 11
    STAGE_CANDIDATE = 12
    START_BATCH = 13
    START_TRIAL = 14
    HEARTBEAT = 15
    STOP = 16
    CLEAR_FAULT = 17
    END_BATCH = 18
    ACK = 128
    INFO = 129
    STATUS = 130
    CONFIG = 131
    TELEMETRY = 144
    RESULT = 145
    FAULT = 146
    NACK = 255


class State(IntEnum):
    IDLE_ZERO = 0
    PRECHECK = 1
    PREPARE_BASELINE = 2
    SETTLE = 3
    APPLY_CANDIDATE = 4
    RUN = 5
    COMPLETE_ZERO = 6
    STOPPED_ZERO = 7
    FAULT_LATCHED_ZERO = 8


class Fault(IntEnum):
    NONE = 0
    HEARTBEAT = 1
    FEEDBACK = 2
    POSITION = 3
    SPEED = 4
    CURRENT = 5
    TEMPERATURE = 6
    PAIR_MISMATCH = 7
    OTHER_JOINT = 8
    PREPARE_TIMEOUT = 9
    SETTLE_TIMEOUT = 10
    CONTROL_TIMING = 11
    BATCH_TIMEOUT = 12
    NONFINITE = 13
    TRANSPORT = 14


class Error(IntEnum):
    BAD_SESSION = 1
    REPLAY = 2
    BAD_LENGTH = 3
    BAD_STATE = 4
    INVALID_CONFIG = 5
    INVALID_PARAMS = 6
    NOT_READY = 7
    INVALID_TRIAL = 8
    BUDGET = 9
    UNKNOWN_COMMAND = 10


class Classification(IntEnum):
    COMPLETE = 0
    STOPPED = 1
    PROTECTION = 2
    PREPARATION_FAILED = 3


def uint(value, name="integer", positive=False):
    if type(value) is not int or not (int(positive) <= value <= 0xffffffff):
        raise ValueError(f"{name}: expected {'positive ' if positive else ''}u32")
    return value


def finite(value, name="number"):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name}: expected finite number, not bool")
    try:
        rounded = struct.unpack("<f", struct.pack("<f", value))[0]
    except (OverflowError, struct.error):
        raise ValueError(f"{name}: outside binary32 range") from None
    if not math.isfinite(rounded):
        raise ValueError(f"{name}: outside binary32 range")
    return value


class Record:
    FORMAT = ""

    def pack(self):
        return struct.pack(self.FORMAT, *(getattr(self, f.name) for f in fields(self)))

    @classmethod
    def unpack(cls, data):
        if len(data) != struct.calcsize(cls.FORMAT):
            raise ValueError(f"{cls.__name__}: wrong payload length")
        return cls(*struct.unpack(cls.FORMAT, data))

    @classmethod
    def from_dict(cls, obj):
        names = {f.name for f in fields(cls)}
        if type(obj) is not dict or set(obj) != names:
            raise ValueError(f"{cls.__name__}: exact fields required: {sorted(names)}")
        return cls(**obj)


@dataclass(frozen=True)
class Safety(Record):
    pos_min: float
    pos_max: float
    margin: float
    max_speed: float
    max_output: float
    max_current: float
    max_temperature: float
    max_pair_error: float
    prepare_speed: float
    prepare_accel: float
    settle_pos_tol: float
    settle_speed_tol: float
    other_pos_tol: float
    other_speed_tol: float
    other_target_1: float
    other_target_2: float
    other_target_3: float
    gain_min_pos_p: float
    gain_min_pos_d: float
    gain_min_speed_p: float
    gain_min_speed_i: float
    gain_max_pos_p: float
    gain_max_pos_d: float
    gain_max_speed_p: float
    gain_max_speed_i: float
    max_integral: float
    feedback_timeout_ms: int
    heartbeat_timeout_ms: int
    prepare_timeout_ms: int
    settle_timeout_ms: int
    settle_hold_ms: int
    max_trial_ms: int
    telemetry_period_ms: int
    batch_timeout_ms: int
    max_control_gap_us: int
    FORMAT = "<26f9I"

    def validate(self):
        names = [f.name for f in fields(self)]
        for name in names[:26]:
            value = finite(getattr(self, name), name)
            if abs(value) > 1e6:
                raise ValueError(name + ": exceeds arithmetic magnitude guard")
        positive = names[3:14] + ["max_integral"]
        if any(getattr(self, n) <= 0 for n in positive) or self.margin < 0:
            raise ValueError("limits/tolerances must be positive; margin nonnegative")
        if self.pos_min + self.margin >= self.pos_max - self.margin:
            raise ValueError("empty position interior")
        if self.prepare_speed > self.max_speed or self.max_output > 32767 or self.max_integral > self.max_output:
            raise ValueError("invalid speed/output/integral limit")
        for suffix in ("pos_p", "pos_d", "speed_p", "speed_i"):
            lo, hi = getattr(self, "gain_min_" + suffix), getattr(self, "gain_max_" + suffix)
            if not 0 <= lo <= hi:
                raise ValueError("invalid gain range: " + suffix)
        for name in names[26:]:
            value = uint(getattr(self, name), name, True)
            if name.endswith("_ms") and value > 1800000:
                raise ValueError(name + ": must be <= 1,800,000 ms")
        if self.feedback_timeout_ms > 60000 or self.heartbeat_timeout_ms > 60000:
            raise ValueError("feedback/heartbeat timeout too large")
        if self.telemetry_period_ms < 5 or self.settle_hold_ms > self.settle_timeout_ms or self.max_trial_ms > self.batch_timeout_ms:
            raise ValueError("invalid telemetry/hold/trial timing")
        if not 1000 <= self.max_control_gap_us <= 100000:
            raise ValueError("invalid control gap")
        # Validate the actual rounded representation too: a tiny interval must not collapse.
        rounded = self.unpack(self.pack())
        if rounded.pos_min + rounded.margin >= rounded.pos_max - rounded.margin:
            raise ValueError("position interior collapses in binary32")
        if rounded != self:
            rounded.validate()  # reject underflow/rounding that changes a limit's validity
        return self

    def contains(self, position):
        return self.pos_min + self.margin <= position <= self.pos_max - self.margin


@dataclass(frozen=True)
class Parameters(Record):
    version: int
    pos_p: float
    pos_d: float
    speed_p: float
    speed_i: float
    algorithm: int
    FORMAT = "<I4fI"

    def validate(self, safety):
        uint(self.version, "version", True)
        if type(self.algorithm) is not int or self.algorithm not in (0, 1):
            raise ValueError("algorithm must be legacy=0 or conditional=1")
        for name in ("pos_p", "pos_d", "speed_p", "speed_i"):
            value = finite(getattr(self, name), name)
            if not getattr(safety, "gain_min_" + name) <= value <= getattr(safety, "gain_max_" + name):
                raise ValueError(name + ": outside safety gain range")
        rounded, envelope = self.unpack(self.pack()), safety.unpack(safety.pack())
        for name in ("pos_p", "pos_d", "speed_p", "speed_i"):
            if not getattr(envelope, "gain_min_" + name) <= getattr(rounded, name) <= getattr(envelope, "gain_max_" + name):
                raise ValueError(name + ": rounded parameter outside wire envelope")
        return self


@dataclass(frozen=True)
class Status(Record):
    state: int
    fault: int
    active_version: int
    applied_seq: int
    sample_seq: int
    trial_id: int
    batch_id: int
    flags: int
    tick_us: int
    tx_dropped: int
    rx_dropped: int
    last_command_seq: int
    FORMAT = "<12I"


@dataclass(frozen=True)
class Telemetry(Record):
    tick_us: int
    sample_seq: int
    trial_id: int
    state: int
    fault: int
    active_version: int
    applied_seq: int
    tx_dropped: int
    rx_dropped: int
    control_dt_us: int
    feedback_age0_ms: int
    feedback_age1_ms: int
    feedback_count0: int
    feedback_count1: int
    flags: int
    target_pos: float
    target_vel: float
    pos0: float
    pos1: float
    vel0: float
    vel1: float
    current0: float
    current1: float
    temperature0: float
    temperature1: float
    pos_error: float
    speed_error: float
    integral: float
    output: float
    command0: float
    command1: float
    FORMAT = "<15I16f"


@dataclass(frozen=True)
class Trial(Record):
    trial_id: int
    mode: int
    origin: float
    target: float
    duration_ms: int
    FORMAT = "<IIffI"

    def validate(self, safety):
        uint(self.trial_id, "trial_id", True)
        uint(self.duration_ms, "duration_ms", True)
        finite(self.origin, "origin")
        finite(self.target, "target")
        if type(self.mode) is not int or self.mode not in (0, 1) or not safety.contains(self.origin):
            raise ValueError("invalid trial mode/origin")
        if self.duration_ms > safety.max_trial_ms:
            raise ValueError("duration exceeds budget")
        if (self.mode == 0 and not safety.contains(self.target)) or (self.mode == 1 and abs(self.target) > safety.max_speed):
            raise ValueError("target outside approved limits")
        return self


@dataclass(frozen=True)
class Result(Record):
    trial_id: int
    classification: int
    fault: int
    run_start_seq: int
    run_end_seq: int
    run_start_us: int
    run_end_us: int
    # status is encoded separately by encode_result/decode_result
    FORMAT = "<7I"


def encode_result(result, status):
    return result.pack() + status.pack()


def decode_result(payload):
    if len(payload) != 76:
        raise ValueError("RESULT: wrong payload length")
    return Result.unpack(payload[:28]), Status.unpack(payload[28:])


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


@dataclass(frozen=True)
class Frame:
    type: int
    boot_id: int = 0
    session: int = 0
    seq: int = 0
    payload: bytes = b""

    def encode(self):
        for name in ("boot_id", "session", "seq"):
            uint(getattr(self, name), name)
        if type(self.type) not in (int, Message) or not 0 <= self.type <= 255:
            raise ValueError("invalid message ID")
        if not isinstance(self.payload, bytes) or len(self.payload) > MAX_PAYLOAD:
            raise ValueError("payload exceeds 256 bytes or is not bytes")
        body = HEADER.pack(SYNC, VERSION, self.type, len(self.payload), self.boot_id, self.session, self.seq) + self.payload
        return body + struct.pack("<H", crc16(body[2:]))


class StreamDecoder:
    """Byte-wise bounded resync. Caller supplies monotonic milliseconds, even on idle reads."""
    def __init__(self):
        self.buffer = bytearray()
        self.pending_since = None
        self.rejected = 0
        self.dropped = 0
        self.last_ms = None

    def feed(self, data=b"", now_ms=0):
        if self.last_ms is not None and now_ms < self.last_ms:
            raise ValueError("decoder clock moved backwards")
        self.last_ms = now_ms
        out = []
        # Incrementally append: memory stays <= MAX_FRAME even for untrusted huge input.
        for byte in data:
            out.extend(self._parse(now_ms))
            self.buffer.append(byte)
            out.extend(self._parse(now_ms))
        out.extend(self._parse(now_ms))
        return out

    def _discard(self):
        del self.buffer[0]
        self.pending_since = None
        self.dropped += 1

    def _parse(self, now):
        result = []
        while self.buffer:
            if self.pending_since is not None and now - self.pending_since >= FRAME_TIMEOUT_MS:
                self.rejected += 1
                self._discard()
                continue
            if self.buffer[0] != SYNC[0] or (len(self.buffer) >= 2 and self.buffer[:2] != SYNC):
                self._discard()
                continue
            if self.pending_since is None:
                self.pending_since = now
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, length, boot, session, seq = HEADER.unpack_from(self.buffer)
            if version != VERSION or length > MAX_PAYLOAD:
                self.rejected += 1
                self._discard()
                continue
            size = HEADER.size + length + 2
            if len(self.buffer) < size:
                break
            raw = self.buffer[:size]
            if crc16(raw[2:-2]) != struct.unpack_from("<H", raw, size - 2)[0]:
                self.rejected += 1
                self._discard()
                continue
            result.append(Frame(kind, boot, session, seq, bytes(raw[HEADER.size:-2])))
            del self.buffer[:size]
            self.pending_since = None
        return result


class CounterUnwrapper:
    """Reject duplicate/stale/reordered u32 samples; permit wrap below half range."""
    def __init__(self):
        self.last = None
        self.value = None

    def add(self, raw):
        uint(raw)
        if self.last is None:
            self.last = self.value = raw
            return self.value
        delta = (raw - self.last) & 0xffffffff
        if not 0 < delta < 0x80000000:
            raise ValueError("stale/reordered counter")
        self.last = raw
        self.value += delta
        return self.value
