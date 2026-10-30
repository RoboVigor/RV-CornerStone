"""constraints_first_v2: validate complete wire windows before any performance score."""
from dataclasses import asdict, dataclass
import math
import statistics
from .protocol import Message as M, State as S, Telemetry, Status, Trial, CounterUnwrapper, decode_result
from .config import SCORE_VERSION


@dataclass
class Score:
    valid: bool
    classification: str
    feasible: object = None
    utility: object = None
    metrics: object = None
    reasons: object = None
    definition: str = SCORE_VERSION

    def __post_init__(self):
        if self.valid and self.utility is not None:
            require(math.isfinite(self.utility), "nonfinite scoring utility")

    def to_dict(self):
        return asdict(self)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def telemetry_health(row, safety):
    """Observer checks use only the explicitly admitted wire safety envelope."""
    s = type(safety).unpack(safety.pack())
    require(s.contains(row.pos0) and s.contains(row.pos1), "position exceeds configured interior")
    require(max(abs(row.vel0), abs(row.vel1), abs(row.target_vel)) <= s.max_speed, "speed/reference exceeds configured limit")
    require(max(abs(row.current0), abs(row.current1)) <= s.max_current, "current exceeds configured limit")
    require(max(row.temperature0, row.temperature1) <= s.max_temperature, "temperature exceeds configured limit")
    require(abs(row.pos0 - row.pos1) <= s.max_pair_error, "motor pair position mismatch")
    require(max(row.feedback_age0_ms, row.feedback_age1_ms) <= s.feedback_timeout_ms, "stale motor feedback")
    require(abs(row.output) <= s.max_output and abs(row.command0) + abs(row.command1) <= s.max_output, "output exceeds configured group limit")
    require(abs(row.integral) <= s.max_integral, "integral exceeds configured limit")
    require(0 < row.control_dt_us <= s.max_control_gap_us, "control timing invalid")
    if row.state in (S.IDLE_ZERO, S.COMPLETE_ZERO, S.STOPPED_ZERO, S.FAULT_LATCHED_ZERO):
        require(not row.flags & 2 and row.output == row.command0 == row.command1 == 0, "zero state contains nonzero command/enable")


def validate_window(frames, expected, safety, performance=None):
    """Validate raw-frame identity/order/endpoints and post-RESULT zero + queried status.

    expected: boot_id, session, trial (dict), version, batch_id. Frames are ordered
    wire frames captured strictly after START submission, through zero-status query.
    """
    trial = Trial.from_dict(expected["trial"])
    version = expected["version"]
    results, run, zeros, status_queries = [], [], [], []
    previous = None
    for index, frame in enumerate(frames):
        require(frame.boot_id == expected["boot_id"] and frame.session == expected["session"], "boot/session mismatch")
        if frame.type == M.FAULT:
            raise ValueError("FAULT in trial window")
        if frame.type == M.RESULT:
            require(frame.seq == 0, "RESULT must be asynchronous")
            result, status = decode_result(frame.payload)
            require(result.trial_id == trial.trial_id, "RESULT trial mismatch")
            require(result.classification == 0 and not result.fault and not status.fault, "non-complete/fault RESULT")
            require(status.state == S.COMPLETE_ZERO and not status.flags & 16 and status.active_version == version, "RESULT not complete-zero/version")
            require(status.batch_id == expected["batch_id"] and status.flags & 8 and status.trial_id == trial.trial_id, "RESULT batch/trial authorization mismatch")
            require(status.applied_seq == result.run_start_seq and status.sample_seq == result.run_end_seq and status.tick_us == result.run_end_us, "RESULT sampled status mismatch")
            results.append((index, result))
        if frame.type == M.STATUS:
            require(frame.seq > 0, "STATUS must confirm an explicit query")
            status = Status.unpack(frame.payload)
            status_queries.append((index, status))
        if frame.type != M.TELEMETRY:
            continue
        require(frame.seq == 0, "TELEMETRY must be asynchronous")
        row = Telemetry.unpack(frame.payload)
        require(all(math.isfinite(v) for v in list(row.__dict__.values())[15:]), "nonfinite telemetry")
        require(not row.fault, "telemetry fault")
        telemetry_health(row, safety)
        require(row.control_dt_us > 0 and row.control_dt_us <= safety.max_control_gap_us, "control timing invalid")
        if previous is not None:
            delta = (row.sample_seq - previous.sample_seq) & 0xffffffff
            dt = (row.tick_us - previous.tick_us) & 0xffffffff
            require(0 < delta < 0x80000000 and 0 < dt < 0x80000000, "duplicate/reordered/reset telemetry")
        previous = row
        if row.state == S.RUN:
            require(row.trial_id == trial.trial_id and row.active_version == version, "RUN trial/version mismatch")
            require(row.flags & 8 and not row.flags & 4, "RUN flags invalid")
            run.append((index, row))
        elif row.state == S.COMPLETE_ZERO and row.trial_id == trial.trial_id:
            require(row.active_version == version and not row.flags & 2 and row.output == row.command0 == row.command1 == 0, "post-result zero invalid")
            zeros.append((index, row))
    require(len(results) == 1 and len(run) >= 2, "missing/duplicate RESULT or RUN window")
    result_index, result = results[0]
    require(run[-1][0] < result_index, "RESULT before last RUN endpoint")
    require(run[0][1].sample_seq == result.run_start_seq and run[-1][1].sample_seq == result.run_end_seq, "missing RUN sample endpoints")
    require(all(row.applied_seq == result.run_start_seq for _, row in run), "RUN applied sample mismatch")
    require(run[0][1].tick_us == result.run_start_us and run[-1][1].tick_us == result.run_end_us, "missing RUN time endpoints")
    last = run[-1][1]
    require(all(row.flags & 2 for _, row in run[:-1]), "premature disabled RUN sample")
    require(not last.flags & 2 and last.output == last.command0 == last.command1 == 0, "final RUN must be disabled/zero")
    post = [(i, row) for i, row in zeros if i > result_index]
    require(post, "missing next COMPLETE_ZERO telemetry")
    require((post[0][1].sample_seq - last.sample_seq) & 0xffffffff == 1, "missing immediate next zero sample")
    clean_status = [(i, s) for i, s in status_queries if i > post[0][0] and s.state == S.COMPLETE_ZERO
                    and not s.fault and not s.flags & 16 and s.flags & 8 and s.batch_id == expected["batch_id"]
                    and s.active_version == version and s.trial_id == trial.trial_id and s.applied_seq == result.run_start_seq
                    and (s.sample_seq - post[0][1].sample_seq) & 0xffffffff < 0x80000000
                    and (s.tick_us - post[0][1].tick_us) & 0xffffffff < 0x80000000]
    require(clean_status, "missing clean post-zero GET_STATUS confirmation")
    # A later status/fault cannot be hidden behind an earlier clean query.
    for index, status in status_queries:
        if index > post[0][0]:
            require(not status.fault and not status.flags & 16 and status.state == S.COMPLETE_ZERO, "late post-RESULT fault/state")
    rows = [row for _, row in run]
    time_counter, sample_counter = CounterUnwrapper(), CounterUnwrapper()
    times = [time_counter.add(row.tick_us) for row in rows]
    samples = [sample_counter.add(row.sample_seq) for row in rows]
    duration_us = times[-1] - times[0]
    require(trial.duration_ms * 1000 <= duration_us <= trial.duration_ms * 1000 + safety.max_control_gap_us, "partial/overlong RUN duration")
    gaps = [b - a for a, b in zip(times, times[1:])]
    period = safety.telemetry_period_ms * 1000
    inferred_missing = sum(max(0, (gap + safety.max_control_gap_us) // period - 1) for gap in gaps)
    tx_lost = (rows[-1].tx_dropped - rows[0].tx_dropped) & 0xffffffff
    rx_lost = (rows[-1].rx_dropped - rows[0].rx_dropped) & 0xffffffff
    require(tx_lost < 0x80000000 and rx_lost < 0x80000000, "drop counter reset/reorder")
    missing = max(inferred_missing, tx_lost)
    fraction = missing / (len(rows) + missing)
    if performance is not None:
        require(max(gaps) <= performance["max_telemetry_gap_ms"] * 1000, "telemetry gap exceeds accepted limit")
        require(fraction <= performance["max_missing_fraction"] and rx_lost == 0, "telemetry loss exceeds accepted limit")
    return trial, rows, [(t - times[0]) / 1000000 for t in times], dict(missing_fraction=fraction, max_gap_ms=max(gaps) / 1000)


def evaluate_trial(frames, expected, safety, performance=None):
    try:
        trial, rows, times, quality = validate_window(frames, expected, safety, performance)
        diagnostics = dict(position_error_abs_max=max(abs(r.pos_error) for r in rows),
                           speed_error_abs_max=max(abs(r.speed_error) for r in rows),
                           integral_abs_max=max(abs(r.integral) for r in rows),
                           output_abs_max=max(abs(r.output) for r in rows),
                           saturated_fraction=sum(bool(r.flags & 1) for r in rows) / len(rows),
                           feedback_age_max_ms=max(max(r.feedback_age0_ms, r.feedback_age1_ms) for r in rows),
                           control_dt_min_us=min(r.control_dt_us for r in rows),
                           control_dt_max_us=max(r.control_dt_us for r in rows))
        measured = [r.vel0 if trial.mode == 1 else r.pos0 for r in rows]
        excitation = trial.target - (0 if trial.mode == 1 else trial.origin)
        diagnostics.update(response_unit="rad/s" if trial.mode == 1 else "rad", final_reference=trial.target,
                           final_error_abs=abs(measured[-1] - trial.target),
                           directional_overshoot=max(0, max((1 if excitation > 0 else -1) * (v - trial.target) for v in measured)) if excitation else None)
        quality["diagnostics"] = diagnostics
        if performance is None:
            return Score(True, "COMPLETE", metrics=quality, reasons=["Gate B: unscored protected trial; descriptive diagnostics only"])
        speed = trial.mode == 1
        suffix = "rad_s" if speed else "rad"
        reference = trial.target
        values = [r.vel0 if speed else r.pos0 for r in rows]
        errors = [abs(v - reference) for v in values]
        band = performance["final_error_" + suffix]
        direction = 1 if reference - (0 if speed else trial.origin) > 0 else -1
        amplitude = abs(reference - (0 if speed else trial.origin))
        require(amplitude > 0, "zero excitation cannot be scored")
        reference_ready = 0
        if speed:
            reference_ready = next((i for i, r in enumerate(rows) if abs(r.target_vel - reference) <= max(1e-7, abs(reference) * 1e-6)), None)
            require(reference_ready is not None, "speed final reference never reached")
            require(all(abs(r.target_vel - reference) <= max(1e-7, abs(reference) * 1e-6) for r in rows[reference_ready:]), "speed reference left final value")
        else:
            require(all(abs(r.target_pos - reference) <= max(1e-7, abs(reference) * 1e-6) for r in rows), "position reference mismatch")
        # Stable suffix, not first crossing. Entire suffix must remain in band through end.
        last_bad = max([i for i, error in enumerate(errors) if i < reference_ready or error > band], default=-1)
        first_stable = last_bad + 1
        hold = performance["hold_ms"] / 1000
        settled = first_stable < len(rows) and times[-1] - times[first_stable] >= hold
        settling_ms = times[first_stable] * 1000 if settled else None
        overshoot = max(0.0, max(direction * (value - reference) for value in values))
        tail_start = max(times[reference_ready], times[-1] - max(hold, times[-1] / 4))
        tail = [v for v, t in zip(values, times) if t >= tail_start]
        tail_span = max(tail) - min(tail)
        constraints = {
            "overshoot": overshoot / performance["max_overshoot_" + suffix],
            "final_error": errors[-1] / band,
            "tail_oscillation": tail_span / performance["sustained_oscillation_limit_" + suffix],
            "settling": settling_ms / performance["max_settling_ms"] if settled else 2.0,
        }
        require(all(math.isfinite(value) for value in constraints.values()), "nonfinite normalized scoring constraint")
        feasible = settled and all(value <= 1.0 for value in constraints.values())
        iae = sum((a + b) * 0.5 * (tb - ta) for a, b, ta, tb in zip(errors, errors[1:], times, times[1:]))
        variation = sum(abs(b.command0 - a.command0) + abs(b.command1 - a.command1)
                        for a, b in zip(rows, rows[1:]))
        duration = times[-1]
        cost = ((settling_ms / 1000 if settled else duration + hold) / duration
                + iae / (amplitude * duration) + variation / (safety.max_output * max(1, len(rows) - 1))) / 3
        violation = sum(max(0, v - 1) for v in constraints.values())
        require(all(math.isfinite(value) for value in (iae, variation, cost, violation)), "nonfinite numerical scoring cost/violation")
        utility = 1 / (1 + cost) if feasible else -1 - violation / (1 + violation)
        metrics = dict(quality, unit="rad/s" if speed else "rad", final_reference=reference,
                       reference_ready_ms=times[reference_ready] * 1000, overshoot=overshoot,
                       final_error=errors[-1], tail_peak_to_peak=tail_span, settling_ms=settling_ms,
                       iae=iae, output_variation=variation, normalized_cost=cost, constraint_ratios=constraints)
        return Score(True, "COMPLETE", feasible, utility, metrics, [k for k, v in constraints.items() if v > 1])
    except (ValueError, OverflowError, IndexError, ZeroDivisionError) as exc:
        return Score(False, "TELEMETRY_INVALID", reasons=[str(exc)])


def aggregate(scores, repeats, scenario_count):
    require(len(scores) == repeats * scenario_count and all(s.valid and s.utility is not None and math.isfinite(s.utility) for s in scores), "partial/invalid/nonfinite candidate cannot be registered")
    groups = [[scores[r * scenario_count + s].utility for r in range(repeats)] for s in range(scenario_count)]
    try:
        stats = [dict(mean=statistics.mean(g), stdev=statistics.stdev(g) if len(g) > 1 else None) for g in groups]
    except OverflowError as exc:
        raise ValueError("nonfinite numerical aggregate statistics") from exc
    require(all(math.isfinite(v) for stat in stats for v in stat.values() if v is not None), "nonfinite numerical aggregate statistics")
    return dict(valid=True, feasible=all(s.feasible for s in scores), utility=min(s.utility for s in scores),
                aggregation="worst_repeat_and_scenario_v1", repeats=repeats,
                scenario_statistics=stats,
                insufficient_repeats=repeats < 2, uncertainty="descriptive_only_no_confidence_claim",
                requires_independent_validation=True)
