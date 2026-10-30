"""Offline-only authoritative raw replay. Never opens a device or resumes authorization."""
import json
from pathlib import Path
from .config import Configuration, reject_constant, unique_object
from .protocol import StreamDecoder, Parameters, Trial, Message as M
from .recording import digest, canonical
from .scoring import evaluate_trial, aggregate, Score
from .optimization import BayesianProposer


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"), parse_constant=reject_constant, object_pairs_hook=unique_object)


def rows(path):
    if not path.exists():
        return []
    return [json.loads(line, parse_constant=reject_constant, object_pairs_hook=unique_object)
            for line in path.read_text(encoding="utf-8").splitlines() if line]


def replay(directory, *, propose=False):
    root = Path(directory)
    manifest = read_json(root / "manifest.json")
    expected_files = manifest["files"]
    actual_files = {p.name for p in root.iterdir() if p.is_file() and p.name != "manifest.json"}
    if actual_files != set(expected_files):
        raise ValueError("log manifest file set mismatch; incomplete/edited evidence")
    for name, checksum in expected_files.items():
        if Path(name).name != name or digest((root / name).read_bytes()) != checksum:
            raise ValueError("log integrity failure: " + name)
    config = Configuration.load(root / "config.json")
    meta, summary = read_json(root / "meta.json"), read_json(root / "summary.json")
    config_hash = digest(canonical(config.raw))
    if config_hash != meta["config_sha256"]:
        raise ValueError("config provenance hash mismatch")
    decoder, tx_decoder, frames, tx_frames, last_clock = StreamDecoder(), StreamDecoder(), [], [], 0
    for record in rows(root / "raw.jsonl"):
        last_clock = record["clock_ms"]
        if record["direction"] == "rx":
            frames.extend(decoder.feed(bytes.fromhex(record["hex"]), last_clock))
        elif record["direction"] == "tx":
            tx_frames.extend(tx_decoder.feed(bytes.fromhex(record["hex"]), last_clock))
        else:
            raise ValueError("unknown raw record direction")
    decoder.feed(b"", last_clock + 100)
    tx_decoder.feed(b"", last_clock + 100)
    staged, baselines, starts = {}, {}, {}
    for frame in tx_frames:
        key = (frame.boot_id, frame.session)
        if frame.type in (M.SET_BASELINE, M.STAGE_CANDIDATE):
            parameters = Parameters.unpack(frame.payload)
            (baselines if frame.type == M.SET_BASELINE else staged)[key + (parameters.version,)] = parameters
        elif frame.type == M.START_TRIAL:
            trial = Trial.unpack(frame.payload)
            starts[key + (trial.trial_id,)] = trial
    frame_records = rows(root / "frames.jsonl")
    framing_ok = not decoder.rejected and not tx_decoder.rejected and len(frame_records) == len(frames)
    if framing_ok:
        framing_ok = all(record == dict(index=i, type=frame.type, boot_id=frame.boot_id, session=frame.session,
                                       seq=frame.seq, payload_hex=frame.payload.hex())
                         for i, (record, frame) in enumerate(zip(frame_records, frames), 1))
    result_records = rows(root / "results.jsonl")
    scores, outcomes = {}, []
    for record in result_records:
        trial_id = record["expected"]["trial"]["trial_id"]
        if record.get("complete") and framing_ok:
            if record.get("config_sha256") != config_hash:
                raise ValueError("trial configuration provenance mismatch")
            identity = (record["expected"]["boot_id"], record["expected"]["session"])
            parameters = Parameters.from_dict(record["parameters"])
            applied = staged.get(identity + (record["expected"]["version"],))
            baseline = baselines.get(identity + (record["baseline_version"],))
            actual_trial = starts.get(identity + (trial_id,))
            if applied != parameters or baseline is None or baseline.pack()[4:] != config.baseline.pack()[4:]:
                raise ValueError("effective parameter/baseline provenance differs from raw TX transactions")
            if actual_trial != Trial.from_dict(record["expected"]["trial"]):
                raise ValueError("trial provenance differs from raw START transaction")
            start, end = record["start_cursor"], record["end_cursor"]
            if not 0 <= start < end <= len(frames):
                raise ValueError("invalid trial frame cursors")
            score = evaluate_trial(frames[start:end], record["expected"], config.safety,
                                   config.raw["gate_c"]["performance"] if meta["operation"] == "optimize" else None)
        else:
            score = Score(False, record.get("classification", "COMMUNICATION_INVALID"), reasons=["incomplete trial or raw framing invalid"])
        scores[trial_id] = score
        outcomes.append(dict(trial_id=trial_id, effective_parameters=record["parameters"], score=score.to_dict()))
    observations = []
    for record in rows(root / "observations.jsonl"):
        if record.get("config_sha256") != config_hash:
            raise ValueError("observation configuration mismatch")
        trial_scores = [scores.get(i) for i in record["trial_ids"]]
        if not framing_ok or any(s is None or not s.valid or s.utility is None for s in trial_scores):
            continue
        c = config.raw["gate_c"]
        outcome = aggregate(trial_scores, c["budget"]["repeats"], len(c["scenarios"]))
        if canonical(outcome) != canonical(record["outcome"]):
            raise ValueError("recorded/replayed aggregate mismatch")
        observations.append(dict(record, outcome=outcome))
    report = dict(mode="offline_replay", run_dir=str(root.resolve()), raw_framing_valid=framing_ok, integrity_valid=True,
                  config_sha256=config_hash, score_version=meta["score_version"], stage=meta["stage"],
                  run_classification=summary["classification"], trials=outcomes, observations=observations,
                  requires_independent_validation=True, physical_validation=False, motion_resumed=False)
    if propose:
        if not observations or observations[0]["kind"] != "accepted_baseline":
            raise ValueError("no complete baseline observation; cannot suggest")
        proposer = BayesianProposer(config)
        for record in observations:
            proposer.register(Parameters.from_dict(record["parameters"]), record["outcome"])
        report["offline_next_proposal"] = proposer.suggest()
    return report


def compare_baselines(raw_directory, conditional_directory):
    left, right = replay(raw_directory), replay(conditional_directory)
    a = Configuration.load(Path(raw_directory) / "config.json")
    b = Configuration.load(Path(conditional_directory) / "config.json")
    if a.baseline.algorithm != 0 or b.baseline.algorithm != 1:
        raise ValueError("comparison requires legacy=0 then conditional=1 recordings")
    comparable = (a.raw["environment"] == b.raw["environment"] and a.raw["identity"] == b.raw["identity"]
                  and a.raw["safety"] == b.raw["safety"] and a.raw["gate_c"] == b.raw["gate_c"]
                  and all(getattr(a.baseline, n) == getattr(b.baseline, n) for n in ("pos_p", "pos_d", "speed_p", "speed_i")))
    return dict(mode="baseline_diagnostic_comparison", comparable_configurations=comparable,
                legacy=left, conditional=right, causal_improvement_established=False,
                note="recorded diagnostics only; no automatic baseline acceptance or physical improvement claim",
                requires_independent_validation=True)
