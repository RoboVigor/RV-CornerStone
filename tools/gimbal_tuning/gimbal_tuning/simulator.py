"""Deterministic 1 ms binary device/plant. Offline evidence, NOT a physical model certification."""
from collections import deque
import math
import struct
from .protocol import (Message as M, State as S, Fault as F, Error as E, Classification as C,
                       Frame, StreamDecoder, Safety, Parameters, Trial, Status, Telemetry,
                       Result, encode_result, uint)

ZERO_STATES = {S.IDLE_ZERO, S.COMPLETE_ZERO, S.STOPPED_ZERO, S.FAULT_LATCHED_ZERO}
MOTION_STATES = {S.PRECHECK, S.PREPARE_BASELINE, S.SETTLE, S.APPLY_CANDIDATE, S.RUN}
EMPTY_COMMANDS = {M.GET_STATUS, M.GET_CONFIG, M.HEARTBEAT, M.CLEAR_FAULT, M.END_BATCH}


def clamp(value, limit):
    return max(-limit, min(limit, value))


class SimulatedDevice:
    """Call receive(bytes), advance(milliseconds), drain(). All public messages are wire bytes.

    Fault injection: feedback_enabled[2], feedback_age_ms[2], temperatures[2],
    currents[2], pair_offset, other_positions/speeds/targets/ages (three each).
    The plant is intentionally a small deterministic synthetic load.
    """
    def __init__(self, boot_id=1):
        uint(boot_id, "boot_id", True)
        self.boot_id = boot_id
        self.session_counter = 0
        self.session = 0
        self.decoder = StreamDecoder()
        self.outbox = deque()
        self.tx_dropped = 0
        self.ms = 0
        self.sample_seq = 0
        self.state = S.IDLE_ZERO
        self.fault = F.NONE
        self.safety = self.baseline = self.candidate = self.active = None
        self.baseline_record = None
        self.max_version = 0
        self.applied_seq = 0
        self.last_command = None
        self.last_command_seq = 0
        self.last_error = 0
        self.last_response = None
        self.complete_sample_pending = False
        self.heartbeat_ms = None
        self.batch_id = 0
        self.batch_start_ms = 0
        self.max_trials = self.trials_started = self.last_trial_id = 0
        self.trial = None
        self.trial_id = 0
        self.phase_ms = self.settle_since = self.run_start_ms = 0
        self.run_start_seq = 0
        self.pos = self.vel = self.integral = self.previous_error = 0.0
        self.target_pos = self.target_vel = self.output = 0.0
        self.prepare_ref = self.prepare_velocity = 0.0
        self.saturated = False
        self.feedback_enabled = [True, True]
        self.feedback_age_ms = [0, 0]
        self.feedback_counts = [0, 0]
        self.temperatures = [25.0, 25.0]
        self.currents = [0.0, 0.0]
        self.pair_offset = 0.0
        self.other_positions = [0.0] * 3
        self.other_speeds = [0.0] * 3
        self.other_targets = [0.0] * 3
        self.other_ages = [0] * 3
        self.other_feedback_enabled = [True] * 3

    @property
    def motion(self):
        return self.state in MOTION_STATES

    @property
    def tick_us(self):
        return (self.ms * 1000) & 0xffffffff

    def status(self):
        flags = bool(self.safety) | (bool(self.baseline) << 1) | (bool(self.candidate) << 2) | (bool(self.batch_id) << 3) | (self.motion << 4)
        return Status(int(self.state), int(self.fault), self.active.version if self.active else 0,
                      self.applied_seq, self.sample_seq, self.trial_id, self.batch_id, flags,
                      self.tick_us, self.tx_dropped, self.decoder.dropped, self.last_command_seq)

    def _emit(self, kind, payload, seq=0):
        if len(self.outbox) >= 4096:
            # Simulated bounded UART queue: dropped telemetry is exposed, never silent.
            if kind == M.TELEMETRY:
                self.tx_dropped += 1
                return
            for i, frame in enumerate(self.outbox):
                if frame.type == M.TELEMETRY:
                    del self.outbox[i]
                    self.tx_dropped += 1
                    break
            else:
                self.tx_dropped += 1
                return
        self.outbox.append(Frame(kind, self.boot_id, self.session, seq, payload))

    def drain(self):
        raw = b"".join(frame.encode() for frame in self.outbox)
        self.outbox.clear()
        return raw

    def receive(self, data):
        for frame in self.decoder.feed(data, self.ms):
            self._command(frame)

    def _ack(self, command, error=0):
        self._emit(M.NACK if error else M.ACK, struct.pack("<II", command.type, error) + self.status().pack(), command.seq)

    def _zero(self, state):
        self.state = state
        self.output = 0.0
        self.integral = 0.0
        self.target_vel = 0.0
        self.saturated = False

    def _cancel(self, classification, state):
        self.batch_id = 0
        self.candidate = None
        self.complete_sample_pending = False
        self._zero(state)
        if self.trial is not None:
            self._result(classification)
        self.trial = None

    def _fault(self, fault):
        self.fault = fault
        self._cancel(C.PREPARATION_FAILED if self.state in {S.PRECHECK, S.PREPARE_BASELINE, S.SETTLE} else C.PROTECTION, S.FAULT_LATCHED_ZERO)
        self._emit(M.FAULT, self.status().pack())

    def _command(self, cmd):
        if cmd.type == M.STOP:
            if cmd.payload:
                self._ack(cmd, E.BAD_LENGTH)
                return
            self._cancel(C.STOPPED, S.FAULT_LATCHED_ZERO if self.fault else S.STOPPED_ZERO)
            self.session = 0
            self.last_command = None
            self.last_command_seq = 0
            self.heartbeat_ms = None
            self._ack(cmd)
            return
        if cmd.type == M.HELLO:
            if cmd.payload or cmd.boot_id or cmd.session or cmd.seq:
                self._ack(cmd, E.BAD_LENGTH)
                return
            self._cancel(C.STOPPED, S.FAULT_LATCHED_ZERO if self.fault else S.IDLE_ZERO)
            if self.session_counter == 0xffffffff:
                self.session = 0  # fail closed; no session wrap
                self._ack(cmd, E.BAD_SESSION)
                return
            self.session_counter += 1
            self.session = self.session_counter
            self.last_command = None
            self.last_command_seq = 0
            self.last_error = 0
            self.max_version = max(self.active.version if self.active else 0, self.baseline_record.version if self.baseline_record else 0)
            self.heartbeat_ms = None
            info = struct.pack("<8I", 1, 65536, 3, 0x205, 0x206, 500000, 1000, 124)
            self._emit(M.INFO, info + self.status().pack(), cmd.seq)
            return
        if not self.session or cmd.boot_id != self.boot_id or cmd.session != self.session or not cmd.seq:
            self._ack(cmd, E.BAD_SESSION)
            return
        if cmd.seq <= self.last_command_seq:
            if cmd == self.last_command:
                # Replay the original response bytes, never refreshed status/liveness.
                if self.last_response is not None:
                    self._emit(self.last_response.type, self.last_response.payload, cmd.seq)
            else:
                self._ack(cmd, E.REPLAY)
            return
        error = self._execute(cmd)
        # Authenticated rejected attempts consume sequence numbers as well.
        self.last_command, self.last_command_seq, self.last_error = cmd, cmd.seq, error
        if error:
            self._ack(cmd, error)
            self.last_response = self.outbox[-1]
            return
        if cmd.type == M.GET_STATUS:
            self._emit(M.STATUS, self.status().pack(), cmd.seq)
        elif cmd.type == M.GET_CONFIG:
            payload = struct.pack("<I", bool(self.safety)) + (self.safety.pack() if self.safety else bytes(140))
            payload += struct.pack("<I", bool(self.baseline)) + (self.baseline.pack() if self.baseline else bytes(24))
            self._emit(M.CONFIG, payload, cmd.seq)
        else:
            self._ack(cmd)
        self.last_response = self.outbox[-1]

    def _execute(self, cmd):
        kind, data = cmd.type, cmd.payload
        if kind in EMPTY_COMMANDS and data:
            return E.BAD_LENGTH
        if kind in (M.GET_STATUS, M.GET_CONFIG):
            return 0
        if kind == M.HEARTBEAT:
            self.heartbeat_ms = self.ms
            return 0
        if kind == M.SET_SAFETY_CONFIG:
            if len(data) != 140:
                return E.BAD_LENGTH
            if self.motion or self.batch_id:
                return E.BAD_STATE
            try:
                safety = Safety.unpack(data).validate()
            except ValueError:
                return E.INVALID_CONFIG
            self.safety = safety
            # Old gains may be outside a new envelope; require explicit reconfiguration.
            self.baseline = self.candidate = None
            return 0
        if kind in (M.SET_BASELINE, M.STAGE_CANDIDATE):
            if len(data) != 24:
                return E.BAD_LENGTH
            if self.motion or self.fault or (kind == M.SET_BASELINE and self.batch_id):
                return E.BAD_STATE
            if not self.safety:
                return E.NOT_READY
            try:
                params = Parameters.unpack(data).validate(self.safety)
                reinstall = kind == M.SET_BASELINE and params == self.baseline_record
                if (params.version <= self.max_version and not reinstall) or (kind == M.STAGE_CANDIDATE and self.baseline and params.algorithm != self.baseline.algorithm):
                    raise ValueError("parameter version/algorithm")
            except ValueError:
                return E.INVALID_PARAMS
            if kind == M.SET_BASELINE:
                self.baseline = self.baseline_record = params
                self.candidate = None
            else:
                self.candidate = params
            self.max_version = max(self.max_version, params.version)
            return 0
        if kind == M.START_BATCH:
            if len(data) != 8:
                return E.BAD_LENGTH
            if self.motion or self.batch_id or self.fault:
                return E.BAD_STATE
            batch, count = struct.unpack("<II", data)
            if not batch or not count:
                return E.BUDGET
            if not self.safety or not self.baseline or not self.candidate or not self._heartbeat_fresh() or self._health_fault():
                return E.NOT_READY
            if self.candidate.algorithm != self.baseline.algorithm:
                return E.INVALID_PARAMS
            self.batch_id, self.max_trials = batch, count
            self.batch_start_ms = self.ms
            self.trials_started = self.last_trial_id = 0
            return 0
        if kind == M.START_TRIAL:
            if len(data) != 20:
                return E.BAD_LENGTH
            if self.complete_sample_pending:
                return E.NOT_READY  # preserve required post-RESULT COMPLETE_ZERO sample
            if self.motion or self.fault:
                return E.BAD_STATE
            if not self.batch_id or not self.candidate or not self._heartbeat_fresh():
                return E.NOT_READY
            if self.trials_started >= self.max_trials:
                return E.BUDGET
            try:
                trial = Trial.unpack(data).validate(self.safety)
                if trial.trial_id <= self.last_trial_id:
                    raise ValueError("trial ID replay")
            except ValueError:
                return E.INVALID_TRIAL
            self.trial = trial
            self.trial_id = self.last_trial_id = trial.trial_id
            self.trials_started += 1
            self.run_start_seq = self.run_start_ms = 0
            self.phase_ms = self.ms
            self.state = S.PRECHECK
            return 0
        if kind == M.END_BATCH:
            if self.motion:
                return E.BAD_STATE
            self.batch_id = 0
            self.candidate = None
            self._zero(S.FAULT_LATCHED_ZERO if self.fault else S.IDLE_ZERO)
            return 0
        if kind == M.CLEAR_FAULT:
            if self.state != S.FAULT_LATCHED_ZERO:
                return E.BAD_STATE
            if not self.safety or not self._heartbeat_fresh() or self._health_fault():
                return E.NOT_READY
            self.fault = F.NONE
            self.candidate = None
            self._zero(S.IDLE_ZERO)
            return 0
        return E.UNKNOWN_COMMAND

    def _heartbeat_fresh(self):
        return self.safety is not None and self.heartbeat_ms is not None and self.ms - self.heartbeat_ms <= self.safety.heartbeat_timeout_ms

    def _health_fault(self):
        s = self.safety
        values = [self.pos, self.vel, self.pair_offset, self.output, self.integral, *self.temperatures,
                  *self.currents, *self.other_positions, *self.other_speeds, *self.other_targets]
        if not all(math.isfinite(v) for v in values):
            return F.NONFINITE
        if any(age > s.feedback_timeout_ms for age in self.feedback_age_ms):
            return F.FEEDBACK
        if not s.contains(self.pos) or not s.contains(self.pos + self.pair_offset):
            return F.POSITION
        if abs(self.vel) > s.max_speed:
            return F.SPEED
        if any(abs(v) > s.max_current for v in self.currents):
            return F.CURRENT
        if any(v > s.max_temperature for v in self.temperatures):
            return F.TEMPERATURE
        if abs(self.pair_offset) > s.max_pair_error:
            return F.PAIR_MISMATCH
        targets = (s.other_target_1, s.other_target_2, s.other_target_3)
        for i in range(3):
            if (self.other_ages[i] > s.feedback_timeout_ms or abs(self.other_positions[i] - targets[i]) > s.other_pos_tol
                    or abs(self.other_targets[i] - targets[i]) > s.other_pos_tol or abs(self.other_speeds[i]) > s.other_speed_tol):
                return F.OTHER_JOINT
        return F.NONE

    def advance(self, milliseconds=1):
        if type(milliseconds) is not int or milliseconds < 0:
            raise ValueError("advance requires nonnegative integer ms")
        for _ in range(milliseconds):
            self.ms += 1
            self.sample_seq = (self.sample_seq + 1) & 0xffffffff
            self.decoder.feed(b"", self.ms)
            for i in range(2):
                if self.feedback_enabled[i]:
                    self.feedback_age_ms[i] = 0
                    self.feedback_counts[i] = (self.feedback_counts[i] + 1) & 0xffffffff
                else:
                    self.feedback_age_ms[i] += 1
            for i in range(3):
                self.other_ages[i] = 0 if self.other_feedback_enabled[i] else self.other_ages[i] + 1
            if self.batch_id:
                fault = self._health_fault()
                if not self._heartbeat_fresh():
                    fault = F.HEARTBEAT
                elif self.ms - self.batch_start_ms >= self.safety.batch_timeout_ms:
                    fault = F.BATCH_TIMEOUT
                if fault:
                    self._fault(fault)
            emitted = self._step()
            # Plant continues inertial coast in all zero states.
            self.vel += (self.output * 0.01 - 4.0 * self.vel) * 0.001
            self.pos += self.vel * 0.001
            if self.safety and not emitted and self.ms % self.safety.telemetry_period_ms == 0:
                self._telemetry()

    def _apply(self, parameters):
        self.active = parameters
        self.applied_seq = self.sample_seq
        self.integral = 0.0
        self.previous_error = self.target_pos - self.pos

    def _step(self):
        if self.complete_sample_pending:
            self.complete_sample_pending = False
            self._telemetry()
            return True
        if not self.motion:
            self.output = 0.0
            return False
        s, t = self.safety, self.trial
        if self.state == S.PRECHECK:
            self.prepare_ref = self.pos
            self.prepare_velocity = 0.0
            self.target_pos = self.pos
            self._apply(self.baseline)
            self.state, self.phase_ms = S.PREPARE_BASELINE, self.ms
        elif self.state == S.PREPARE_BASELINE:
            if self.ms - self.phase_ms >= s.prepare_timeout_ms:
                self._fault(F.PREPARE_TIMEOUT)
                return False
            delta = t.origin - self.prepare_ref
            desired = math.copysign(min(s.prepare_speed, math.sqrt(2 * s.prepare_accel * abs(delta))), delta) if delta else 0.0
            self.prepare_velocity += clamp(desired - self.prepare_velocity, s.prepare_accel * 0.001)
            increment = self.prepare_velocity * 0.001
            if abs(increment) >= abs(delta):
                self.prepare_ref, self.prepare_velocity = t.origin, 0.0
            else:
                self.prepare_ref += increment
            self.target_pos = self.prepare_ref
            if self.prepare_ref == t.origin:
                self.state, self.phase_ms, self.settle_since = S.SETTLE, self.ms, None
        elif self.state == S.SETTLE:
            self.target_pos = t.origin
            settled = (abs(self.pos - t.origin) <= s.settle_pos_tol and abs(self.pos + self.pair_offset - t.origin) <= s.settle_pos_tol and abs(self.vel) <= s.settle_speed_tol)
            if settled:
                if self.settle_since is None:
                    self.settle_since = self.ms
                if self.ms - self.settle_since >= s.settle_hold_ms:
                    self.state = S.APPLY_CANDIDATE
            else:
                self.settle_since = None
            if self.state == S.SETTLE and self.ms - self.phase_ms >= s.settle_timeout_ms:
                self._fault(F.SETTLE_TIMEOUT)
                return False
        if self.state == S.APPLY_CANDIDATE:
            # APPLY uses the very same sampled-at-rest snapshot that completed hold.
            self.target_pos = t.origin
            self._apply(self.candidate)
            self.candidate = None
            self.state = S.RUN
            self.run_start_ms, self.run_start_seq = self.ms, self.sample_seq
        if self.state == S.RUN:
            self.target_pos = t.target if t.mode == 0 else t.origin
            self.target_vel = clamp(t.target, s.prepare_accel * (self.ms - self.run_start_ms) / 1000) if t.mode == 1 else self._position_control()
        else:
            self.target_vel = self._position_control()
        error = self.target_vel - self.vel
        proposed = clamp(self.integral + self.active.speed_i * error, s.max_integral)
        unsaturated = self.active.speed_p * error + proposed
        if self.active.algorithm == 0 or abs(unsaturated) <= s.max_output or unsaturated * error <= 0:
            self.integral = proposed
        raw = self.active.speed_p * error + self.integral
        self.output = clamp(raw, s.max_output)
        self.saturated = abs(raw) >= s.max_output
        first = self.state == S.RUN and self.ms == self.run_start_ms
        final = self.state == S.RUN and self.ms - self.run_start_ms >= t.duration_ms
        if final:
            self.output = 0.0  # final RUN endpoint is already a real zero command
            self.saturated = False
        if first or final:
            self._telemetry(motion_enabled=not final)
        if final:
            self._zero(S.COMPLETE_ZERO)
            self._result(C.COMPLETE)
            self.trial = None
            self.complete_sample_pending = True
        return first or final

    def _position_control(self):
        error = self.target_pos - self.pos
        value = self.active.pos_p * error + self.active.pos_d * (error - self.previous_error)
        self.previous_error = error
        return clamp(value, self.safety.max_speed)

    def _telemetry(self, motion_enabled=None):
        enabled = self.motion if motion_enabled is None else motion_enabled
        flags = self.saturated | (enabled << 1) | ((self.active == self.baseline and self.active is not None) << 2) | ((self.state == S.RUN) << 3)
        out = self.output if enabled else 0.0
        item = Telemetry(self.tick_us, self.sample_seq, self.trial_id, self.state, self.fault,
                         self.active.version if self.active else 0, self.applied_seq, self.tx_dropped,
                         self.decoder.dropped, 1000, *self.feedback_age_ms, *self.feedback_counts, flags,
                         self.target_pos, self.target_vel, self.pos, self.pos + self.pair_offset,
                         self.vel, self.vel, *self.currents, *self.temperatures, self.target_pos - self.pos,
                         self.target_vel - self.vel, self.integral, out, float(int(out * 0.5)), float(int(out * 0.5)))
        self._emit(M.TELEMETRY, item.pack())

    def _result(self, classification):
        result = Result(self.trial_id, classification, self.fault, self.run_start_seq, self.sample_seq,
                        (self.run_start_ms * 1000) & 0xffffffff, self.tick_us)
        self._emit(M.RESULT, encode_result(result, self.status()))

    def reboot(self, boot_id=None):
        next_id = self.boot_id + 1 if boot_id is None else boot_id
        uint(next_id, "new boot_id", True)
        if next_id == self.boot_id:
            raise ValueError("reboot requires a different nonzero boot_id")
        self.__init__(next_id)


class SimulatedTransport:
    """Serial-like transport; reconnect explicitly requires a fresh HELLO from caller."""
    def __init__(self, device=None):
        self.device = device if device is not None else SimulatedDevice()
        self.closed = False
        self.pending = bytearray()

    def write(self, data):
        if self.closed:
            raise OSError("transport closed")
        self.device.receive(data)
        return len(data)

    def read(self, size=4096):
        if self.closed:
            raise OSError("transport closed")
        self.pending.extend(self.device.drain())
        out = bytes(self.pending[:size])
        del self.pending[:size]
        return out

    def advance(self, milliseconds=1):
        if self.closed:
            raise OSError("transport closed")
        self.device.advance(milliseconds)

    def close(self):
        self.closed = True
        self.pending.clear()
