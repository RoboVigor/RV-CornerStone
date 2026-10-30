import copy
from dataclasses import replace, asdict
from pathlib import Path
from gimbal_tuning.config import Configuration
from gimbal_tuning.protocol import (Frame, Message as M, State as S, Status, Telemetry, Result, Trial, encode_result)

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


def fast_config(stage="position_pd", repeats=1, candidates=2):
    raw = copy.deepcopy(Configuration.load(EXAMPLES / ("simulation-speed.json" if stage == "speed_pi" else "simulation.json")).raw)
    raw["trial"].update(mode=1 if stage == "speed_pi" else 0, target=0.03, duration_ms=250)
    raw["safety"]["settle_hold_ms"] = 5
    c = raw["gate_c"]
    c["performance"].update(hold_ms=20, max_settling_ms=200)
    c["budget"].update(max_candidates=candidates, repeats=repeats)
    c["scenarios"] = [copy.deepcopy(raw["trial"]), dict(raw["trial"], trial_id=2, target=-raw["trial"]["target"])]
    return Configuration(raw).validate("C")


def window(values=None, mode=0, duration=1000, wrap=False):
    config = Configuration.load(EXAMPLES / "simulation.json")
    safety = config.safety
    perf = dict(config.raw["gate_c"]["performance"])
    values = values if values is not None else [min(0.1, i / 300 * 0.1) for i in range(0, duration + 1, 10)]
    seed = Telemetry.unpack(bytes(124))
    origin_seq, origin_time = (0xfffffff0, 0xffff0000) if wrap else (1, 1000)
    frames = []
    for i, value in enumerate(values):
        row = replace(seed, tick_us=(origin_time + i * 10000) & 0xffffffff, sample_seq=(origin_seq + i * 10) & 0xffffffff,
                      trial_id=1, state=S.RUN, active_version=2, applied_seq=origin_seq, control_dt_us=1000,
                      feedback_count0=i + 1, feedback_count1=i + 1, flags=8 if i == len(values) - 1 else 10,
                      target_pos=0 if mode else 0.1, target_vel=min(0.1, i / 20 * 0.1) if mode else 0,
                      pos0=0 if mode else value, pos1=0 if mode else value, vel0=value if mode else 0,
                      output=0 if i == len(values) - 1 else 50, command0=0 if i == len(values) - 1 else 25,
                      command1=0 if i == len(values) - 1 else 25)
        frames.append(Frame(M.TELEMETRY, 9, 3, 0, row.pack()))
    first, last = Telemetry.unpack(frames[0].payload), Telemetry.unpack(frames[-1].payload)
    status = Status(S.COMPLETE_ZERO, 0, 2, origin_seq, last.sample_seq, 1, 1, 11, last.tick_us, 0, 0, 20)
    result = Result(1, 0, 0, first.sample_seq, last.sample_seq, first.tick_us, last.tick_us)
    frames.append(Frame(M.RESULT, 9, 3, 0, encode_result(result, status)))
    zero = replace(last, tick_us=(last.tick_us + 1000) & 0xffffffff, sample_seq=(last.sample_seq + 1) & 0xffffffff,
                   state=S.COMPLETE_ZERO, flags=0, output=0, command0=0, command1=0)
    frames.append(Frame(M.TELEMETRY, 9, 3, 0, zero.pack()))
    frames.append(Frame(M.STATUS, 9, 3, 21, replace(status, sample_seq=zero.sample_seq, tick_us=zero.tick_us).pack()))
    trial = Trial(1, mode, 0, 0.1, duration)
    expected = dict(boot_id=9, session=3, trial=asdict(trial), version=2, batch_id=1)
    return frames, expected, safety, perf
