"""Bounded asynchronous durable records; UDP is nonblocking and never authoritative."""
from dataclasses import asdict, is_dataclass
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import queue
import socket
import struct
import threading
import time


def clean(value):
    if is_dataclass(value):
        return clean(asdict(value))
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)  # raw frame remains authoritative; never serialize a valid numeric NaN
    return value


def canonical(value):
    return json.dumps(clean(value), sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def digest(data):
    return hashlib.sha256(data).hexdigest()


class RunLog:
    """Writer thread isolates fsync/storage latency from the serial/heartbeat owner.

    Queue exhaustion or disk failure is fatal to an experiment, never silent data loss.
    All trial/observation barriers fsync. A final hash manifest detects offline edits.
    """
    def __init__(self, directory, config, metadata=None, queue_size=16384):
        self.path = Path(directory)
        self.path.mkdir(parents=True, exist_ok=False)
        snapshot = canonical(config.raw)
        self.config_hash = digest(snapshot)
        (self.path / "config.json").write_bytes(snapshot + b"\n")
        if config.source_bytes is not None:
            (self.path / "config.original.json").write_bytes(config.source_bytes)
        c = config.raw["gate_c"]
        meta = dict(format_version=1, config_sha256=self.config_hash, source_path=config.source_path,
                    environment=config.raw["environment"], identity=config.raw["identity"],
                    algorithm=config.baseline.algorithm, declared_baseline=clean(config.baseline),
                    score_version=c.get("scoring_version"), stage=c.get("stage"), seed=c.get("random_seed"),
                    budget=c.get("budget"), pid_units="discrete_per_sample", created_unix_ns=time.time_ns(),
                    dependencies={"pyserial": "3.5", "bayesian-optimization": "3.1.0", "packaging": "25.0"},
                    requires_independent_validation=True, physical_validation=False)
        from importlib.metadata import version, PackageNotFoundError
        installed = {}
        for package in ("gimbal-tuning", "pyserial", "bayesian-optimization", "packaging", "numpy", "scipy", "scikit-learn"):
            try:
                installed[package] = version(package)
            except PackageNotFoundError:
                installed[package] = None
        meta["dependencies_installed"] = installed
        meta.update(metadata or {})
        (self.path / "meta.json").write_bytes(canonical(meta) + b"\n")
        import os
        for path in self.path.iterdir():
            with path.open("rb") as handle:
                os.fsync(handle.fileno())
        self.queue = queue.Queue(queue_size)
        self.error = None
        self.closed = False
        self.thread = threading.Thread(target=self._writer, name="gimbal-log", daemon=True)
        self.thread.start()

    def emit(self, name, record):
        if self.closed or self.error:
            raise OSError("run log unavailable: " + str(self.error or "closed"))
        if name not in {"raw", "frames", "telemetry", "results", "observations", "events"}:
            raise ValueError("unknown log channel")
        try:
            self.queue.put_nowait((name, clean(record)))
        except queue.Full:
            raise OSError("bounded log queue exhausted; experiment must stop") from None

    def checkpoint(self, timeout=10):
        event = threading.Event()
        try:
            self.queue.put(("barrier", event), timeout=timeout)
        except queue.Full:
            raise OSError("log barrier queue timeout") from None
        if not event.wait(timeout) or self.error:
            raise OSError("durable log checkpoint failed: " + str(self.error or "timeout"))

    def _writer(self):
        handles = {}
        last_sync = time.monotonic()
        def sync():
            import os
            for handle in handles.values():
                handle.flush()
                os.fsync(handle.fileno())
        try:
            while True:
                try:
                    name, value = self.queue.get(timeout=0.25)
                except queue.Empty:
                    sync()
                    last_sync = time.monotonic()
                    continue
                if name in {"barrier", "close"}:
                    sync()
                    last_sync = time.monotonic()
                    value.set()
                    if name == "close":
                        break
                else:
                    if name not in handles:
                        handles[name] = (self.path / (name + ".jsonl")).open("ab")
                    handles[name].write(canonical(value) + b"\n")
                    if time.monotonic() - last_sync >= 0.25:
                        sync()
                        last_sync = time.monotonic()
        except BaseException as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        finally:
            for handle in handles.values():
                handle.close()

    def close(self, summary):
        if self.closed:
            return
        event = threading.Event()
        try:
            self.queue.put(("close", event), timeout=5)
            if not event.wait(10) or self.error:
                raise OSError("run log finalization failed: " + str(self.error or "timeout"))
            self.thread.join(timeout=1)
            (self.path / "summary.json").write_bytes(canonical(summary) + b"\n")
            files = {p.name: digest(p.read_bytes()) for p in self.path.iterdir() if p.is_file() and p.name != "manifest.json"}
            (self.path / "manifest.json").write_bytes(canonical(dict(format_version=1, files=files)) + b"\n")
            # Metadata/manifest durable too; no successful export precedes this barrier.
            import os
            for path in self.path.iterdir():
                if path.is_file():
                    with path.open("rb") as handle:
                        os.fsync(handle.fileno())
            if os.name != "nt":
                fd = os.open(self.path, os.O_RDONLY)
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
        finally:
            self.closed = True


class VofaForwarder:
    """Read-only best-effort JustFloat. Numeric literal loopback only; no DNS/network targets."""
    def __init__(self, host="127.0.0.1", port=1347):
        address = ipaddress.ip_address(host)
        if not address.is_loopback or type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("VOFA must use numeric loopback and explicit valid UDP port")
        family = socket.AF_INET6 if address.version == 6 else socket.AF_INET
        self.socket = socket.socket(family, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.endpoint = (str(address), port)
        self.dropped = 0

    def send(self, row):
        channels = (row.target_pos, row.pos0, row.pos1, row.target_vel, row.vel0, row.vel1,
                    row.pos_error, row.speed_error, row.integral, row.output, row.command0, row.command1,
                    float(row.state), float(row.fault), float(row.active_version))
        try:
            self.socket.sendto(struct.pack("<15f", *channels) + b"\x00\x00\x80\x7f", self.endpoint)
        except (OSError, OverflowError):
            self.dropped += 1

    def close(self):
        self.socket.close()
