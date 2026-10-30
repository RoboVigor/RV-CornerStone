"""CLI foundations: admission, read-only status, deterministic offline trials."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import struct
import sys
from .config import Configuration
from .protocol import Message as M, State, Classification, Status, Telemetry, decode_result
from .simulator import SimulatedDevice, SimulatedTransport
from .transport import ProtocolClient, SerialTransport, ProtocolError
from .manager import ExperimentManager
from .analysis import replay, compare_baselines
from .recording import clean

EXAMPLES = Path(__file__).resolve().parent / "examples"


def preflight_output_path(output, *evidence_dirs):
    """Resolve aliases before work; derived files must not mutate their evidence."""
    if output is None:
        return None
    destination = Path(output).resolve()
    for directory in evidence_dirs:
        root = Path(directory).resolve()
        if destination.is_relative_to(root):
            raise ValueError(f"output must be outside immutable evidence directory: {root}")
    return destination


def run_simulation(config, mode=None):
    """One explicitly configured trial, through binary transport only; no physical claims.

    Returns raw frames plus decoded RUN rows/RESULT/status for later manager integration.
    Does not perform scoring, BO, VOFA, hardware I/O or auto-resume.
    """
    config.validate("B")
    if config.raw["environment"] != "simulation_only":
        raise ValueError("simulate requires explicitly simulation_only configuration")
    device = SimulatedDevice()
    s = config.safety
    device.other_positions = [s.other_target_1, s.other_target_2, s.other_target_3]
    device.other_targets = list(device.other_positions)
    transport = SimulatedTransport(device)
    raw_frames = []
    client = ProtocolClient(transport, raw_observer=lambda direction, data: raw_frames.append((direction, data)))
    client.connect()
    client.command(M.SET_SAFETY_CONFIG, s.pack())
    client.command(M.SET_BASELINE, config.baseline.pack())
    client.command(M.STAGE_CANDIDATE, config.candidate.pack())
    client.command(M.HEARTBEAT)
    client.command(M.START_BATCH, struct.pack("<II", 1, 1))
    trial = config.trial if mode is None else replace(config.trial, mode=mode).validate(s)
    client.command(M.START_TRIAL, trial.pack())
    interval = max(1, s.heartbeat_timeout_ms // 3)
    result = None
    inspected = 0
    for elapsed in range(s.batch_timeout_ms + 1):
        if elapsed % interval == 0:
            client.command(M.HEARTBEAT)
        transport.advance(1)
        client.poll()
        for frame in client.received[inspected:]:
            if frame.type == M.RESULT:
                result = decode_result(frame.payload)
        inspected = len(client.received)
        if result:
            transport.advance(1)  # explicit next COMPLETE_ZERO sample
            client.poll()
            break
    if not result:
        client.stop()
        raise ProtocolError("simulation budget exhausted without RESULT")
    rows = [Telemetry.unpack(f.payload) for f in client.received if f.type == M.TELEMETRY]
    run = [row for row in rows if row.state == State.RUN and row.trial_id == trial.trial_id]
    summary, result_status = result
    if summary.classification != Classification.COMPLETE:
        raise ProtocolError(f"simulation aborted: classification={summary.classification}, fault={summary.fault}")
    if not run or (run[0].sample_seq, run[-1].sample_seq) != (summary.run_start_seq, summary.run_end_seq):
        raise ProtocolError("incomplete RUN endpoints")
    if any(row.active_version != config.candidate.version for row in run):
        raise ProtocolError("RUN parameter version mismatch")
    if run[-1].command0 or run[-1].command1:
        raise ProtocolError("final RUN endpoint did not record zero commands")
    client.command(M.END_BATCH)
    status = Status.unpack(client.command(M.GET_STATUS).payload)
    if status.state != State.IDLE_ZERO or status.flags & (8 | 16) or device.output:
        raise ProtocolError("simulation did not end zero/unauthorized")
    transport.close()
    return {"trial": trial, "run": run, "telemetry": rows, "result": summary,
            "result_status": result_status, "status": status, "raw_frames": raw_frames}


def main(argv=None):
    parser = argparse.ArgumentParser(description="Explicit offline-first joint0 experiments; hardware motion requires Gate B/C + port + authorization")
    sub = parser.add_subparsers(dest="command", required=True)
    validate = sub.add_parser("validate", help="validate configuration admission without opening a port")
    validate.add_argument("config", type=Path)
    validate.add_argument("--gate", choices=("A", "B", "C"), default="A")
    validate.add_argument("--hardware", action="store_true", help="forbid simulation configs")
    simulate = sub.add_parser("simulate", help="run one synthetic binary position/speed trial")
    simulate.add_argument("--config", type=Path, default=EXAMPLES / "simulation.json")
    simulate.add_argument("--mode", choices=("position", "speed"), default=None)
    simulate.add_argument("--output", type=Path, help="optional offline JSON data, includes raw frame hex")
    status = sub.add_parser("status", help="HELLO + GET_STATUS only; HELLO revokes previous authorization")
    status_mode = status.add_mutually_exclusive_group()
    status_mode.add_argument("--port", help="explicit hardware serial port; omitted = simulator")
    status_mode.add_argument("--simulate", action="store_true")
    for name in ("baseline", "trial", "optimize"):
        command = sub.add_parser(name, help="protected single baseline/trial" if name != "optimize" else "Gate C bounded propose/register search; accepted baseline first")
        command.add_argument("--config", type=Path, required=True)
        command.add_argument("--run-dir", type=Path, required=True, help="new durable run directory (must not exist)")
        mode = command.add_mutually_exclusive_group(required=True)
        mode.add_argument("--simulate", action="store_true")
        mode.add_argument("--port", help="explicit hardware serial port")
        command.add_argument("--authorize-motion", action="store_true", help="explicit human motion authorization; hardware only")
        command.add_argument("--vofa-port", type=int, help="optional best-effort loopback JustFloat UDP port")
        command.add_argument("--vofa-host", default="127.0.0.1", help="numeric loopback only")
        if name == "optimize":
            command.add_argument("--export-best", type=Path, help="unverified candidate export, never loaded automatically")
    for name in ("analyze", "replay"):
        command = sub.add_parser(name, help="offline authoritative raw replay; never reconnects or resumes motion")
        command.add_argument("run_dir", type=Path)
        command.add_argument("--output", type=Path)
        if name == "replay":
            command.add_argument("--propose", action="store_true", help="offline-only replay observations and suggest once")
    comparison = sub.add_parser("compare", help="legacy vs conditional recorded diagnostics, not physical acceptance")
    comparison.add_argument("legacy", type=Path)
    comparison.add_argument("conditional", type=Path)
    comparison.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "optimize":
            args.export_best = preflight_output_path(args.export_best, args.run_dir)
        elif args.command in ("analyze", "replay"):
            args.output = preflight_output_path(args.output, args.run_dir)
        elif args.command == "compare":
            args.output = preflight_output_path(args.output, args.legacy, args.conditional)
        if args.command == "validate":
            config = Configuration.load(args.config).validate(args.gate, hardware=args.hardware)
            print(json.dumps({"valid": True, "gate": args.gate, "environment": config.raw["environment"],
                              "hardware": args.hardware, "physical_safety_validated": False}))
        elif args.command == "simulate":
            config = Configuration.load(args.config)
            mode = {"position": 0, "speed": 1, None: None}[args.mode]
            data = run_simulation(config, mode)
            report = {"environment": "simulation_only", "physical_safety_validated": False,
                      "mode": data["trial"].mode, "run_samples": len(data["run"]),
                      "result": asdict(data["result"]), "ending_status": asdict(data["status"])}
            if args.output:
                record = dict(report, trial=asdict(data["trial"]), telemetry=[asdict(r) for r in data["telemetry"]],
                              raw_frames=[{"direction": d, "hex": raw.hex()} for d, raw in data["raw_frames"]])
                args.output.write_text(json.dumps(record, indent=2, allow_nan=False) + "\n", encoding="utf-8")
            print(json.dumps(report, allow_nan=False))
        elif args.command in ("baseline", "trial", "optimize"):
            if args.simulate and args.authorize_motion:
                raise ValueError("--authorize-motion is hardware-only; simulation must remain explicitly simulation_only")
            config = Configuration.load(args.config)
            manager = ExperimentManager(config, args.run_dir, hardware=bool(args.port), port=args.port,
                                        authorize_motion=args.authorize_motion,
                                        vofa={"host": args.vofa_host, "port": args.vofa_port} if args.vofa_port is not None else None)
            result = manager.run_search() if args.command == "optimize" else manager.run_single(baseline=args.command == "baseline")
            if args.command == "optimize" and args.export_best and result.get("best_candidate"):
                best = result["best_candidate"]
                export = dict(best, run_dir=str(args.run_dir.resolve()), config_sha256=result["config_sha256"],
                              trial_trace=[r for r in result["trials"] if r["expected"]["trial"]["trial_id"] in best["trial_ids"]],
                              evidence_manifest=str((args.run_dir / "manifest.json").resolve()),
                              requires_independent_validation=True, publishable=False)
                args.export_best.write_text(json.dumps(clean(export), indent=2, allow_nan=False) + "\n", encoding="utf-8")
            print(json.dumps(clean(dict(classification=result["classification"], run_dir=str(args.run_dir),
                                        trial_count=len(result["trials"]), observation_count=len(result["observations"]),
                                        zero_confirmed=result["zero_confirmed"], stop_confirmed=result.get("stop_confirmed"),
                                        error=result.get("error"), log_error=result.get("log_error"),
                                        requires_independent_validation=True)), allow_nan=False))
            if result.get("interrupted"):
                return 130
            if result["classification"] == "BUDGET_EXHAUSTED" and result.get("termination") != "candidate_count":
                return 3
            return 0 if result["classification"] in ("COMPLETE", "BUDGET_EXHAUSTED") and result["zero_confirmed"] else 2
        elif args.command in ("analyze", "replay", "compare"):
            report = compare_baselines(args.legacy, args.conditional) if args.command == "compare" else replay(args.run_dir, propose=getattr(args, "propose", False))
            if args.output:
                args.output.write_text(json.dumps(clean(report), indent=2, allow_nan=False) + "\n", encoding="utf-8")
            print(json.dumps(clean(report), allow_nan=False))
        else:
            transport = SerialTransport(args.port) if args.port else SimulatedTransport()
            try:
                client = ProtocolClient(transport, hardware=bool(args.port))
                client.connect()
                result = Status.unpack(client.command(M.GET_STATUS).payload)
                print(json.dumps({"boot_id": client.boot_id, "session": client.session, "status": asdict(result),
                                  "physical_safety_validated": False}))
            finally:
                transport.close()
        return 0
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
