"""Explicit experiments. This module, never BO/VOFA, owns admission and batch motion."""
import copy
from dataclasses import asdict, replace
import struct
from .config import Configuration
from .protocol import Message as M, State as S, Status, Parameters, Trial, Telemetry, decode_result
from .io_owner import IOOwner, ExperimentFailure
from .recording import RunLog, VofaForwarder, canonical
from .scoring import evaluate_trial, aggregate
from .optimization import BayesianProposer, materialize
from .simulator import SimulatedDevice, SimulatedTransport
from .transport import SerialTransport


class ExperimentManager:
    def __init__(self, config, run_dir, *, hardware=False, port=None, authorize_motion=False,
                 transport_factory=None, vofa=None, simulated_tick_s=0.0001, log_factory=RunLog):
        # Snapshot caller-owned dicts: no configuration mutation after admission.
        self.config = Configuration(copy.deepcopy(config.raw), config.source_bytes, config.source_path)
        self.config.validate("B", hardware=hardware)
        if hardware and (not port or authorize_motion is not True):
            raise ValueError("hardware motion requires explicit --port and --authorize-motion")
        if not hardware and config.raw["environment"] != "simulation_only":
            raise ValueError("simulated execution requires simulation_only config")
        self.run_dir, self.hardware, self.port = run_dir, hardware, port
        self.transport_factory, self.vofa_options = transport_factory, vofa
        self.tick_sleep, self.log_factory = simulated_tick_s, log_factory
        self.owner = self.log = self.vofa = None
        self.version = self.trial_counter = 0
        self.baseline = None
        self.batch_id = 1
        self.in_batch = False
        self.completed, self.observations = [], []
        self.current = None
        self.started = False

    def run_single(self, *, baseline=False):
        return self._execute("baseline" if baseline else "trial", lambda: self._single(baseline))

    def run_search(self, proposer=None):
        self.config.validate("C", hardware=self.hardware)
        # Dependency initialization before opening the serial port/authorizing a batch.
        proposer = proposer if proposer is not None else BayesianProposer(self.config)
        return self._execute("optimize", lambda: self._search(proposer))

    def _execute(self, operation, work):
        if self.started:
            raise ValueError("manager instances are single-use; no resume/reconnect")
        self.started = True
        summary = dict(operation=operation, classification="COMMUNICATION_INVALID", trials=[], observations=[],
                       requires_independent_validation=True, physical_validation=False, zero_confirmed=False)
        try:
            self.log = self.log_factory(self.run_dir, self.config, dict(operation=operation, hardware=self.hardware, port=self.port))
            if self.vofa_options:
                self.vofa = VofaForwarder(**self.vofa_options)
            if self.transport_factory:
                transport = self.transport_factory()
            elif self.hardware:
                transport = SerialTransport(self.port, timeout=min(0.01, self.config.safety.heartbeat_timeout_ms / 8000))
            else:
                device = SimulatedDevice()
                s = self.config.safety
                device.other_positions = [s.other_target_1, s.other_target_2, s.other_target_3]
                device.other_targets = list(device.other_positions)
                transport = SimulatedTransport(device)
            if not self.hardware and (isinstance(transport, SerialTransport) or getattr(transport, "is_hardware", False)):
                transport.close()
                raise ValueError("hardware-capable transport cannot be used as simulation")
            try:
                self.owner = IOOwner(transport, self.config.safety, self.log, self.vofa, simulated_tick_s=self.tick_sleep)
            except BaseException:
                transport.close()
                raise
            self._open()
            summary.update(work())
            summary["zero_confirmed"] = True
        except (Exception, KeyboardInterrupt) as exc:
            classification = exc.classification if isinstance(exc, ExperimentFailure) else "STOPPED" if isinstance(exc, KeyboardInterrupt) else "COMMUNICATION_INVALID"
            confirmed = self.owner.stop() if self.owner else False  # no connection is not proof of device zero
            summary.update(classification=classification, error=f"{type(exc).__name__}: {exc}",
                           zero_confirmed=confirmed, stop_confirmed=confirmed if self.owner else None,
                           stop_error=self.owner.stop_error if self.owner else None,
                           requires_manual_estop=bool(self.owner and not confirmed), interrupted=isinstance(exc, KeyboardInterrupt),
                           motion_commands_possible=self.owner is not None)
            if self.current is not None and self.log:
                partial = dict(self.current, classification=classification, score=None, complete=False,
                               end_cursor=self.owner.cursor() if self.owner else None, error=str(exc))
                self.completed.append(partial)
                try:
                    self.log.emit("results", partial)
                except OSError:
                    pass
                self.current = None
        finally:
            if self.owner:
                try:
                    self.owner.close()
                except OSError as exc:
                    summary.update(classification="COMMUNICATION_INVALID", zero_confirmed=False, error=str(exc), requires_manual_estop=True)
                summary.update(heartbeat_count=self.owner.heartbeat_count, max_heartbeat_gap_ms=self.owner.max_heartbeat_gap_ms)
            if self.vofa:
                summary["udp_dropped"] = self.vofa.dropped
                self.vofa.close()
            summary.update(trials=self.completed, observations=self.observations,
                           effective_baseline=asdict(self.baseline) if self.baseline else None,
                           config_sha256=self.log.config_hash if self.log else None)
            if self.log:
                try:
                    self.log.close(summary)
                except OSError as exc:
                    summary.update(classification="COMMUNICATION_INVALID", log_error=str(exc), best_candidate=None)
        return summary

    def _open(self):
        info = self.owner.request(M.HELLO)
        status = Status.unpack(info.payload[32:])
        if status.fault or status.flags & (8 | 16):
            raise ExperimentFailure("PROTECTION", "initial device fault/authorization; manual recovery required")
        config = self.owner.request(M.GET_CONFIG)
        prior_baseline_version = Parameters.unpack(config.payload[148:172]).version
        self.version = max(status.active_version, prior_baseline_version, self.config.baseline.version - 1)
        self.owner.request(M.SET_SAFETY_CONFIG, self.config.safety.pack())
        self.baseline = self._new_version(self.config.baseline)
        self.owner.request(M.SET_BASELINE, self.baseline.pack())
        self.log.emit("events", dict(kind="configured", boot_id=self.owner.boot_id, session=self.owner.session,
                                     declared_baseline=asdict(self.config.baseline), effective_baseline=asdict(self.baseline)))

    def _new_version(self, parameters):
        if self.version >= 0xffffffff:
            raise ExperimentFailure("COMMUNICATION_INVALID", "parameter version exhausted; no automatic new session")
        self.version += 1
        return Parameters.unpack(replace(parameters, version=self.version).pack()).validate(type(self.config.safety).unpack(self.config.safety.pack()))

    def _zero_status(self, *, complete=False):
        frame = self.owner.request(M.GET_STATUS)
        status = Status.unpack(frame.payload)
        states = {S.COMPLETE_ZERO} if complete else {S.IDLE_ZERO, S.COMPLETE_ZERO}
        if status.state not in states or status.fault or status.flags & 16:
            raise ExperimentFailure("PROTECTION", "queried status is not healthy zero")
        if complete and (status.batch_id != self.batch_id or not status.flags & 8):
            raise ExperimentFailure("COMMUNICATION_INVALID", "batch authorization changed")
        return status

    def _begin_batch(self, count, first_parameters, budget_ms):
        if not 0 < count <= 0xffffffff:
            raise ValueError("trial budget exceeds u32")
        self._zero_status()
        self.owner.request(M.STAGE_CANDIDATE, first_parameters.pack())
        self.owner.request(M.HEARTBEAT)
        self.owner.deadline_ms = self.owner.now_ms + budget_ms
        self.owner.request(M.START_BATCH, struct.pack("<II", self.batch_id, count))
        self.in_batch = True

    def _end_batch(self):
        self._zero_status(complete=True)
        self.owner.request(M.END_BATCH)
        status = self._zero_status()
        if status.flags & 8 or status.batch_id:
            raise ExperimentFailure("COMMUNICATION_INVALID", "END_BATCH not confirmed revoked")
        self.owner.deadline_ms = None
        self.in_batch = False

    def _trial(self, scenario, parameters, *, staged=False, candidate_index=0, repeat=0, scenario_index=0, scored=False):
        self.owner.check()
        self._zero_status(complete=bool(self.trial_counter))
        if not staged:
            self.owner.request(M.STAGE_CANDIDATE, parameters.pack())
        self.trial_counter += 1
        trial = Trial.unpack(replace(scenario, trial_id=self.trial_counter).validate(self.config.safety).pack())
        start_cursor = self.owner.cursor()
        expected = dict(boot_id=self.owner.boot_id, session=self.owner.session, trial=asdict(trial),
                        version=parameters.version, batch_id=self.batch_id)
        self.current = dict(expected=expected, parameters=asdict(parameters), baseline_version=self.baseline.version,
                            candidate_index=candidate_index, repeat=repeat, scenario_index=scenario_index,
                            config_sha256=self.log.config_hash, start_cursor=start_cursor)
        start_ms = self.owner.now_ms
        self.owner.request(M.START_TRIAL, trial.pack())
        entries, cursor = [], start_cursor
        result_seen = zero_seen = False
        phase, phase_start, last_run_rx = None, start_ms, None
        s = self.config.safety
        maximum = s.prepare_timeout_ms + s.settle_timeout_ms + trial.duration_ms + 4 * s.telemetry_period_ms + 10
        while True:
            self.owner.check()
            fresh = self.owner.since(cursor, wait=0.01)
            if fresh:
                entries.extend(fresh)
                cursor = fresh[-1][0]
            for _, frame in fresh:
                if frame.type == M.RESULT:
                    result, status = decode_result(frame.payload)
                    if result.trial_id == trial.trial_id:
                        if result.classification != 0 or result.fault or status.fault:
                            raise ExperimentFailure("PROTECTION", "non-complete/fault RESULT")
                        result_seen = True
                elif frame.type == M.TELEMETRY:
                    row = Telemetry.unpack(frame.payload)
                    if row.trial_id != trial.trial_id:
                        if row.state == S.RUN:
                            raise ExperimentFailure("TELEMETRY_INVALID", "unexpected RUN trial identity")
                        continue
                    if row.state == S.RUN and row.active_version != parameters.version:
                        raise ExperimentFailure("TELEMETRY_INVALID", "unexpected RUN parameter version")
                    if row.state in (S.PREPARE_BASELINE, S.SETTLE) and row.active_version != self.baseline.version:
                        raise ExperimentFailure("PROTECTION", "preparation did not use the accepted baseline transaction")
                    if row.state != phase:
                        phase, phase_start = row.state, self.owner.now_ms
                    if row.state == S.RUN:
                        last_run_rx = self.owner.now_ms
                    if result_seen and row.state == S.COMPLETE_ZERO:
                        if row.output or row.command0 or row.command1 or row.flags & 2 or row.fault:
                            raise ExperimentFailure("PROTECTION", "post-result command is not zero")
                        zero_seen = True
            now = self.owner.now_ms
            if result_seen and zero_seen:
                status = self._zero_status(complete=True)
                if status.active_version != parameters.version or status.trial_id != trial.trial_id:
                    raise ExperimentFailure("COMMUNICATION_INVALID", "post-zero version/trial mismatch")
                tail = self.owner.since(cursor)
                entries.extend(tail)
                if tail:
                    cursor = tail[-1][0]
                self.owner.check()
                break
            if now - start_ms > maximum:
                raise ExperimentFailure("PREPARATION_FAILED" if phase != S.RUN else "COMMUNICATION_INVALID", "host per-trial deadline exceeded")
            limits = {S.PRECHECK: s.prepare_timeout_ms, S.PREPARE_BASELINE: s.prepare_timeout_ms, S.SETTLE: s.settle_timeout_ms,
                      S.RUN: trial.duration_ms + 2 * s.telemetry_period_ms}
            if phase in limits and now - phase_start > limits[phase] + 2 * s.telemetry_period_ms:
                raise ExperimentFailure("PREPARATION_FAILED" if phase != S.RUN else "COMMUNICATION_INVALID", "host phase deadline exceeded")
            if scored and last_run_rx is not None and not result_seen and now - last_run_rx > self.config.raw["gate_c"]["performance"]["max_telemetry_gap_ms"] + s.max_control_gap_us / 1000:
                raise ExperimentFailure("TELEMETRY_INVALID", "live RUN telemetry delivery gap exceeded")
        score = evaluate_trial([f for _, f in entries], expected, s, self.config.raw["gate_c"]["performance"] if scored else None)
        record = dict(self.current, end_cursor=cursor, classification=score.classification, complete=score.valid, score=score.to_dict())
        self.completed.append(record)
        self.current = None
        self.log.emit("results", record)
        self.log.checkpoint()
        self.owner.check()
        if not score.valid:
            raise ExperimentFailure(score.classification, "; ".join(score.reasons or []))
        return score, record

    def _single(self, baseline):
        parameters = self._new_version(self.config.baseline if baseline else self.config.candidate)
        self._begin_batch(1, parameters, self.config.safety.batch_timeout_ms)
        self._trial(self.config.trial, parameters, staged=True)
        self._end_batch()
        return dict(classification="COMPLETE", baseline_accepted=False, best_candidate=None,
                    note="protected acquisition only; no physical validation or automatic baseline acceptance")

    def _search(self, proposer):
        c = self.config.raw["gate_c"]
        budget = c["budget"]
        scenarios = [Trial.from_dict(scene) for scene in c["scenarios"]]
        count = budget["max_candidates"] * budget["repeats"] * len(scenarios)
        first = self._new_version(self.baseline)
        self._begin_batch(count, first, budget["max_total_ms"])
        best = None
        for candidate_index in range(budget["max_candidates"]):
            self.owner.check()
            if candidate_index == 0:
                selected = self.baseline  # accepted baseline is always first; no suggestion yet
            else:
                proposal = proposer.suggest()  # owner continues heartbeat and wall-budget supervision
                self.owner.check()
                selected = materialize(self.config, proposal, self.version + 1)
            scores, trace, versions = [], [], []
            for repeat in range(budget["repeats"]):
                for scenario_index, scene in enumerate(scenarios):
                    initially_staged = candidate_index == repeat == scenario_index == 0
                    params = first if initially_staged else self._new_version(selected)
                    score, record = self._trial(scene, params, staged=initially_staged, candidate_index=candidate_index,
                                                repeat=repeat, scenario_index=scenario_index, scored=True)
                    scores.append(score)
                    trace.append(record["expected"]["trial"]["trial_id"])
                    versions.append(params.version)
            outcome = aggregate(scores, budget["repeats"], len(scenarios))
            effective = Parameters.from_dict(self.completed[-len(trace)]["parameters"])
            observation = dict(candidate_index=candidate_index, kind="accepted_baseline" if candidate_index == 0 else "proposal",
                               parameters=asdict(effective), effective_versions=versions,
                               baseline_version=self.baseline.version, trial_ids=trace, outcome=outcome,
                               config_sha256=self.log.config_hash, stage=c["stage"])
            self.owner.check()
            self.log.emit("observations", observation)
            self.log.checkpoint()
            self.owner.check()
            proposer.register(effective, outcome)
            self.observations.append(observation)
            if best is None or outcome["utility"] > best["outcome"]["utility"]:
                best = observation
        self._end_batch()
        return dict(classification="BUDGET_EXHAUSTED", termination="candidate_count", zero_confirmed=True,
                    best_candidate=dict(best, requires_independent_validation=True, publishable=False,
                                        accepted_speed_loop=False, note="unverified export only; never automatically loaded"))
