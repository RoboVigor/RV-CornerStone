"""Single-owner synchronous transport foundation. No reconnect, START retry or background motion."""
import struct
import time
from .protocol import Frame, StreamDecoder, Message as M, Status, Parameters, Trial, Safety


class ProtocolError(RuntimeError):
    pass


class CommandRejected(ProtocolError):
    def __init__(self, command, error, status):
        self.command, self.error, self.status = command, error, status
        super().__init__(f"command {command} rejected: error {error}, state {status.state}")


class SerialTransport:
    """Lazy optional pyserial import; exclusive opening where the platform supports it.

    Opening a port grants no motion. Caller must own it exclusively; no automatic
    reconnect or retry. Windows exclusivity is enforced by the OS serial handle.
    """
    @property
    def is_hardware(self):
        return True

    def __init__(self, port, baudrate=500000, timeout=0.05):
        if not isinstance(port, str) or not port.strip():
            raise ValueError("explicit serial port required")
        if baudrate != 500000 or timeout <= 0:
            raise ValueError("wire v1 requires baud 500000 and bounded positive timeout")
        try:
            import serial
        except ImportError:
            raise RuntimeError("hardware serial requires pyserial==3.5") from None
        import os
        options = dict(port=port, baudrate=baudrate, timeout=timeout, write_timeout=timeout)
        if os.name != "nt":
            options["exclusive"] = True
        self.serial = serial.Serial(**options)

    def write(self, data):
        count = self.serial.write(data)
        if count != len(data):
            raise OSError("partial serial write: command outcome uncertain; inspect status and STOP")
        return count

    def read(self, size=4096):
        return self.serial.read(min(size, max(1, self.serial.in_waiting)))

    def close(self):
        self.serial.close()


class ProtocolClient:
    """Small command session API. Hardware mutations require an admitted config.

    raw_observer(direction, bytes) can persist authoritative frames before decoding.
    poll() returns asynchronous frames; it never interprets telemetry as a heartbeat.
    Create/connect explicitly on reconnect. A timeout never retries a command.
    """
    READ_ONLY = {M.GET_STATUS, M.GET_CONFIG, M.HEARTBEAT}

    def __init__(self, transport, config=None, hardware=False, raw_observer=None, timeout=0.5):
        self.transport = transport
        self.config = config
        self._hardware_requested = bool(hardware)
        self.raw_observer = raw_observer
        self.timeout = timeout
        self.decoder = StreamDecoder()
        self.boot_id = self.session = self.seq = 0
        self.received = []
        self._epoch = time.monotonic()

    @property
    def hardware(self):
        return self._hardware_requested or isinstance(self.transport, SerialTransport) or bool(getattr(self.transport, "is_hardware", False))

    @hardware.setter
    def hardware(self, value):
        self._hardware_requested = bool(value)

    def _send(self, frame):
        data = frame.encode()
        if self.raw_observer:
            self.raw_observer("tx", data)
        if self.transport.write(data) != len(data):
            raise OSError("partial command write; no automatic retry")

    def poll(self):
        data = self.transport.read(4096)
        if data and self.raw_observer:
            self.raw_observer("rx", data)
        frames = self.decoder.feed(data, (time.monotonic() - self._epoch) * 1000)
        accepted = []
        for frame in frames:
            if self.boot_id and (frame.boot_id != self.boot_id or frame.session != self.session):
                raise ProtocolError("boot/session changed: authorization lost; fresh HELLO required")
            accepted.append(frame)
        self.received.extend(accepted)
        return accepted

    def _wait(self, seq, types):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            for frame in self.poll():
                if frame.seq == seq and frame.type == M.NACK:
                    if len(frame.payload) != 56:
                        raise ProtocolError("invalid NACK length")
                    command, error = struct.unpack("<II", frame.payload[:8])
                    raise CommandRejected(command, error, Status.unpack(frame.payload[8:]))
                if frame.seq == seq and frame.type in types:
                    return frame
        raise TimeoutError("uncertain command outcome: no retry; inspect status and safely STOP")

    def connect(self):
        # HELLO revokes any prior authorization, including an active device trial.
        self.boot_id = self.session = self.seq = 0
        self.received.clear()
        self.decoder = StreamDecoder()
        self._send(Frame(M.HELLO))
        frame = self._wait(0, {M.INFO})
        if len(frame.payload) != 80 or not frame.boot_id or not frame.session:
            raise ProtocolError("invalid INFO identity/length")
        identity = struct.unpack("<8I", frame.payload[:32])
        if identity != (1, 65536, 3, 0x205, 0x206, 500000, 1000, 124):
            raise ProtocolError("unsupported device identity")
        self.boot_id, self.session = frame.boot_id, frame.session
        self.received = [item for item in self.received
                         if item.boot_id == self.boot_id and item.session == self.session]
        return frame

    def command(self, kind, payload=b""):
        if kind in (M.HELLO, M.STOP):
            raise ValueError("use connect()/stop() for session barriers")
        if not self.session:
            raise ProtocolError("fresh HELLO required")
        if self.hardware and kind not in self.READ_ONLY:
            if self.config is None:
                raise ValueError("hardware motion/configuration requires Gate B admission")
            self.config.validate("B", hardware=True)
            self._admit_hardware_payload(kind, payload)
        if self.seq == 0xffffffff:
            raise ProtocolError("sequence exhausted; stop and start a new session")
        self.seq += 1
        self._send(Frame(kind, self.boot_id, self.session, self.seq, payload))
        types = {M.GET_STATUS: {M.STATUS}, M.GET_CONFIG: {M.CONFIG}}.get(kind, {M.ACK})
        frame = self._wait(self.seq, types)
        expected = {M.STATUS: 48, M.CONFIG: 172, M.ACK: 56}[frame.type]
        if len(frame.payload) != expected:
            raise ProtocolError("response length mismatch")
        if frame.type == M.ACK:
            command, error = struct.unpack("<II", frame.payload[:8])
            if command != kind or error:
                raise ProtocolError("ACK command/error mismatch")
        return frame

    def _admit_hardware_payload(self, kind, payload):
        """Bind wire values to the reviewed config, not merely a boolean gate flag."""
        if kind == M.SET_SAFETY_CONFIG and payload != self.config.safety.pack():
            raise ValueError("safety payload differs from admitted configuration")
        if kind == M.SET_BASELINE:
            Parameters.unpack(payload).validate(Safety.unpack(self.config.safety.pack()))
            if payload[4:] != self.config.baseline.pack()[4:]:
                raise ValueError("baseline gains/algorithm differ from admitted configuration")
        if kind == M.START_BATCH:
            if len(payload) != 8:
                raise ValueError("invalid batch payload")
            batch, count = struct.unpack("<II", payload)
            if not batch or not count:
                raise ValueError("positive batch/count required")
            if count > 1:
                self.config.validate("C", hardware=True)
                c = self.config.raw["gate_c"]
                maximum = c["budget"]["max_candidates"] * c["budget"]["repeats"] * len(c["scenarios"])
                if count > maximum:
                    raise ValueError("batch exceeds admitted trial budget")
        if kind == M.STAGE_CANDIDATE:
            params = Parameters.unpack(payload).validate(Safety.unpack(self.config.safety.pack()))
            if params.algorithm != self.config.baseline.algorithm:
                raise ValueError("candidate algorithm has no accepted baseline")
            if payload[4:] not in (self.config.candidate.pack()[4:], self.config.baseline.pack()[4:]):
                self.config.validate("C", hardware=True)
                for name, (lo, hi) in self.config.raw["gate_c"]["search"].items():
                    if not lo <= getattr(params, name) <= hi:
                        raise ValueError("candidate exceeds admitted search bounds")
        if kind == M.START_TRIAL:
            trial = Trial.unpack(payload).validate(self.config.safety)
            if payload[4:] != self.config.trial.pack()[4:]:
                self.config.validate("C", hardware=True)
                # Trial ID may increase between repetitions; scenario motion values may not.
                scenarios = self.config.raw["gate_c"]["scenarios"]
                if not any((trial.mode, trial.origin, trial.target, trial.duration_ms) ==
                           (item.mode, item.origin, item.target, item.duration_ms)
                           for item in (Trial.unpack(Trial.from_dict(s).pack()) for s in scenarios)):
                    raise ValueError("trial differs from admitted scenarios")

    def stop(self):
        # STOP works even before HELLO and after session/boot uncertainty.
        self.boot_id = self.session = 0
        self._send(Frame(M.STOP))
        frame = self._wait(0, {M.ACK})
        if len(frame.payload) != 56 or struct.unpack("<II", frame.payload[:8]) != (M.STOP, 0) or frame.session:
            raise ProtocolError("invalid STOP confirmation")
        status = Status.unpack(frame.payload[8:])
        if status.flags & (8 | 16) or status.state not in (7, 8):
            raise ProtocolError("STOP not confirmed zero/unauthorized")
        self.seq = 0
        return frame
