"""Fail-closed JSON admission. No physical limit or acceptance has a default."""
from dataclasses import dataclass
import json
from pathlib import Path
import struct
from .protocol import Safety, Parameters, Trial, VERSION, finite, uint

SCORE_VERSION = "constraints_first_v2"
STAGE_FIELDS = {"speed_pi": ("speed_p", "speed_i"), "position_pd": ("pos_p", "pos_d")}
CONFIRMATIONS = (
    "human_supervision", "physical_support", "independent_estop", "coast_margin_verified",
    "fixed_pose_verified", "motor_mapping_verified", "units_verified", "direction_zero_verified",
    "firmware_protocol_verified", "initialization_policy_accepted", "small_trajectory_accepted",
    "conditional_trial_authorized", "conditional_baseline_accepted", "clock_rng_verified",
)
IDENTITY_FIELDS = {"protocol", "firmware", "motor0_id", "motor1_id", "baud", "nominal_control_us",
                   "angle_unit", "speed_unit", "current_unit", "temperature_unit", "output_unit",
                   "pid_sample_semantics", "initialization_policy"}
PERFORMANCE_FIELDS = {"max_overshoot_rad", "final_error_rad", "sustained_oscillation_limit_rad",
                      "max_overshoot_rad_s", "final_error_rad_s", "sustained_oscillation_limit_rad_s",
                      "hold_ms", "max_settling_ms", "max_telemetry_gap_ms", "max_missing_fraction"}
BUDGET_FIELDS = {"max_candidates", "max_total_ms", "max_trial_ms", "repeats", "termination_rule"}
SEARCH_FIELDS = {"pos_p", "pos_d", "speed_p", "speed_i"}
ROOT_FIELDS = {"schema_version", "environment", "identity", "confirmations", "safety", "baseline", "candidate", "trial", "gate_c"}


def f32(value):
    return struct.unpack("<f", struct.pack("<f", finite(value)))[0]


def wire_bounds(lo, hi):
    """Inward-rounded nonnegative binary32 interval; never widen accepted bounds."""
    finite(lo); finite(hi)
    if not 0 <= lo <= hi <= 1e6:
        raise ValueError("invalid search range")
    lower, upper = f32(lo), f32(hi)
    if lower < lo:
        lower = struct.unpack("<f", struct.pack("<I", struct.unpack("<I", struct.pack("<f", lower))[0] + 1))[0]
    if upper > hi:
        upper = struct.unpack("<f", struct.pack("<I", struct.unpack("<I", struct.pack("<f", upper))[0] - 1))[0]
    if lower > upper:
        raise ValueError("search interval has no binary32 value")
    return lower, upper


def exact(obj, names, label):
    if type(obj) is not dict or set(obj) != set(names):
        raise ValueError(f"{label}: exact fields required: {sorted(names)}")


def positive(value, label):
    if finite(value, label) <= 0:
        raise ValueError(label + ": must be positive")


def reject_constant(value):
    raise ValueError("JSON nonfinite constant forbidden: " + value)


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate JSON key: " + key)
        result[key] = value
    return result


def reject_nonfinite_tree(value):
    if type(value) is float:
        finite(value)
    elif isinstance(value, dict):
        for child in value.values():
            reject_nonfinite_tree(child)
    elif isinstance(value, list):
        for child in value:
            reject_nonfinite_tree(child)


@dataclass(frozen=True)
class Configuration:
    raw: dict
    source_bytes: bytes = None
    source_path: str = None

    @classmethod
    def load(cls, path):
        source = Path(path).read_bytes()
        raw = json.loads(source.decode("utf-8"), parse_constant=reject_constant, object_pairs_hook=unique_object)
        config = cls(raw, source, str(Path(path).resolve()))
        config.validate("A")
        return config

    @property
    def safety(self):
        return Safety.from_dict(self.raw["safety"]).validate()

    @property
    def baseline(self):
        return Parameters.from_dict(self.raw["baseline"]).validate(self.safety)

    @property
    def candidate(self):
        return Parameters.from_dict(self.raw["candidate"]).validate(self.safety)

    @property
    def trial(self):
        return Trial.from_dict(self.raw["trial"]).validate(self.safety)

    def validate(self, gate="A", hardware=False):
        if gate not in ("A", "B", "C"):
            raise ValueError("gate must be A, B or C")
        exact(self.raw, ROOT_FIELDS, "configuration")
        reject_nonfinite_tree(self.raw)
        if type(self.raw["schema_version"]) is not int or self.raw["schema_version"] != 1:
            raise ValueError("unsupported configuration schema")
        if self.raw["environment"] not in ("simulation_only", "hardware"):
            raise ValueError("explicit simulation_only/hardware environment required")
        if hardware and self.raw["environment"] != "hardware":
            raise ValueError("simulation configuration is forbidden on hardware")
        exact(self.raw["identity"], IDENTITY_FIELDS, "identity")
        exact(self.raw["confirmations"], CONFIRMATIONS, "confirmations")
        for name, confirmed in self.raw["confirmations"].items():
            if type(confirmed) is not bool:
                raise ValueError(name + ": explicit boolean required")
        if gate == "A":
            return self
        if not all(v for n, v in self.raw["confirmations"].items() if n not in ("conditional_trial_authorized", "conditional_baseline_accepted")):
            raise ValueError("Gate B: all explicit human/physical confirmations required")
        identity = self.raw["identity"]
        for name, value in dict(protocol=VERSION, firmware=65536, motor0_id=0x205, motor1_id=0x206,
                                baud=500000, nominal_control_us=1000).items():
            if type(identity[name]) is not int or identity[name] != value:
                raise ValueError("identity mismatch/unconfirmed: " + name)
        for name, value in dict(angle_unit="rad", speed_unit="rad/s", current_unit="A", temperature_unit="degC",
                                output_unit="esc_command", pid_sample_semantics="discrete_per_sample",
                                initialization_policy="reset_at_rest_v1").items():
            if identity[name] != value:
                raise ValueError("unit/policy mismatch/unconfirmed: " + name)
        safety, baseline, candidate, trial = self.safety, self.baseline, self.candidate, self.trial
        if candidate.version <= baseline.version or candidate.algorithm != baseline.algorithm:
            raise ValueError("candidate must have newer version and same algorithm")
        if hardware and baseline.algorithm == 1 and self.raw["confirmations"]["conditional_trial_authorized"] is not True:
            raise ValueError("conditional algorithm requires explicit protected trial authorization")
        if gate == "B":
            return self
        if baseline.algorithm == 1 and self.raw["confirmations"]["conditional_baseline_accepted"] is not True:
            raise ValueError("Gate C conditional algorithm requires independently accepted baseline")
        c = self.raw["gate_c"]
        exact(c, {"baseline_accepted", "preparation_accepted", "performance_accepted", "search_accepted",
                  "budget_accepted", "performance", "search", "budget", "scoring_version", "scenarios",
                  "random_seed", "parameter_units", "firmware", "stage", "speed_loop_accepted"}, "gate_c")
        for name in ("baseline_accepted", "preparation_accepted", "performance_accepted", "search_accepted", "budget_accepted"):
            if c[name] is not True:
                raise ValueError("Gate C requires explicit acceptance: " + name)
        if c["stage"] not in STAGE_FIELDS or type(c["speed_loop_accepted"]) is not bool:
            raise ValueError("explicit speed_pi/position_pd stage and speed_loop_accepted required")
        if c["stage"] == "position_pd" and c["speed_loop_accepted"] is not True:
            raise ValueError("position PD requires independently accepted baseline speed P/I")
        exact(c["performance"], PERFORMANCE_FIELDS, "performance")
        perf = c["performance"]
        for name in PERFORMANCE_FIELDS - {"hold_ms", "max_settling_ms", "max_telemetry_gap_ms", "max_missing_fraction"}:
            positive(perf[name], name)
        for name in ("hold_ms", "max_settling_ms", "max_telemetry_gap_ms"):
            uint(perf[name], name, True)
        fraction = finite(perf["max_missing_fraction"], "max_missing_fraction")
        if not 0 <= fraction < 1 or perf["hold_ms"] > perf["max_settling_ms"] or perf["max_settling_ms"] > trial.duration_ms:
            raise ValueError("invalid performance/window thresholds")
        if perf["max_telemetry_gap_ms"] < safety.telemetry_period_ms:
            raise ValueError("telemetry gap threshold below configured cadence")
        exact(c["search"], SEARCH_FIELDS, "search")
        for name, limits in c["search"].items():
            if type(limits) is not list or len(limits) != 2:
                raise ValueError(name + ": explicit lower/upper pair required")
            lo, hi = (finite(x, name) for x in limits)
            if not getattr(safety, "gain_min_" + name) <= lo <= hi <= getattr(safety, "gain_max_" + name):
                raise ValueError(name + ": search outside safety envelope")
            if not lo <= getattr(baseline, name) <= hi or not lo <= getattr(candidate, name) <= hi:
                raise ValueError(name + ": baseline/candidate outside search")
            if name in STAGE_FIELDS[c["stage"]]:
                lower, upper = wire_bounds(lo, hi)
                if lower >= upper:
                    raise ValueError(name + ": active axis needs nonzero binary32 width")
                if not lower <= f32(getattr(baseline, name)) <= upper or not lower <= f32(getattr(candidate, name)) <= upper:
                    raise ValueError(name + ": rounded baseline/candidate outside admitted search")
            elif lo != hi or lo != getattr(baseline, name) or getattr(candidate, name) != getattr(baseline, name):
                raise ValueError(name + ": inactive gains must equal accepted baseline")
        exact(c["budget"], BUDGET_FIELDS, "budget")
        budget = c["budget"]
        for name in BUDGET_FIELDS - {"termination_rule"}:
            uint(budget[name], name, True)
        if budget["max_trial_ms"] > safety.max_trial_ms or trial.duration_ms > budget["max_trial_ms"] or budget["max_total_ms"] > safety.batch_timeout_ms:
            raise ValueError("host budget exceeds device safety budget")
        if budget["termination_rule"] != "stop_on_fault_or_budget":
            raise ValueError("unsupported/unconfirmed termination rule")
        uint(c["random_seed"], "random_seed")
        if type(c["firmware"]) is not int or c["firmware"] != identity["firmware"]:
            raise ValueError("Gate C firmware mismatch")
        if c["scoring_version"] != SCORE_VERSION or c["parameter_units"] != "discrete_per_sample":
            raise ValueError("scoring/parameter semantics unconfirmed")
        if type(c["scenarios"]) is not list or not c["scenarios"]:
            raise ValueError("explicit nonempty scenarios required")
        count = budget["max_candidates"] * budget["repeats"] * len(c["scenarios"])
        if not 0 < count <= 0xffffffff:
            raise ValueError("candidate/repeat/scenario product must fit positive u32")
        directions = set()
        for scenario in c["scenarios"]:
            item = Trial.from_dict(scenario).validate(safety)
            if item.duration_ms > budget["max_trial_ms"] or item.duration_ms < perf["max_settling_ms"]:
                raise ValueError("scenario conflicts with trial/performance budget")
            if item.mode != (1 if c["stage"] == "speed_pi" else 0):
                raise ValueError("scenario mode must match stage")
            wire = Trial.unpack(item.pack())
            excitation = wire.target if wire.mode == 1 else wire.target - wire.origin
            if excitation == 0:
                raise ValueError("search needs nonzero binary32 excitation")
            directions.add(1 if excitation > 0 else -1)
            if item.mode == 1 and abs(item.target) / safety.prepare_accel * 1000 + perf["hold_ms"] > item.duration_ms:
                raise ValueError("speed reference cannot reach final value and hold within RUN")
        if directions != {-1, 1}:
            raise ValueError("Gate C requires explicitly approved bidirectional scenarios")
        return self
