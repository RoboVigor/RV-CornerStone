# Joint0 gimbal tuning host

Python 3.10+, offline-first. Wire source: `../../docs/gimbal-tuning-protocol.md`.
No Flash writes, automatic reconnect/resume, protection fallback, automatic fault clearing,
best-gain loading or default publication. Synthetic tests are **not physical validation**.

## Offline quick start

From repository root:

```sh
cd tools/gimbal_tuning
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
python3 -m gimbal_tuning validate examples/hardware-template.json --gate A --hardware
python3 -m gimbal_tuning validate examples/simulation-speed.json --gate C
python3 -m gimbal_tuning simulate
python3 -m gimbal_tuning simulate --mode speed
python3 -m gimbal_tuning baseline --simulate --config examples/simulation.json --run-dir /tmp/gimbal-baseline-001
python3 -m gimbal_tuning trial --simulate --config examples/simulation.json --run-dir /tmp/gimbal-trial-001
```

Run directories must not already exist. Evidence directories are immutable after sealing:
`optimize --export-best` must be outside its `--run-dir`; `analyze/replay --output` must
be outside the source run; `compare --output` must be outside **both** input runs.
Paths are resolved (including relative components and symlinks) and forbidden outputs
are rejected before configuration loading, manager/port creation or offline analysis.
Keep exports/reports as siblings of run directories, as in the examples below, or use
stdout. Never refresh a sealed manifest to accommodate a derived file.

Standard-library tests require no installation;
only the two real optimizer tests are skipped if its optional dependency is absent.
For imports from repository root use `PYTHONPATH=tools/gimbal_tuning`. Alternatively,
run the module from `tools` (source-tree bridge) or install the package. Both examples
and the empty hardware template are bundled in the installed package; editable copies
are under `examples/`, with tests preventing divergence.

### Windows installation and explicit motion

PowerShell, from `tools/gimbal_tuning`:

```powershell
py -3 -m venv "$env:TEMP\gimbal-tuning-venv"
$python = "$env:TEMP\gimbal-tuning-venv\Scripts\python.exe"
& $python -m pip install ".[serial,optimize]"
& $python -m unittest discover -s tests -v
& $python -m gimbal_tuning status --simulate
& $python -m gimbal_tuning baseline --simulate --config examples/simulation.json --run-dir runs\baseline-sim-001
& $python -m gimbal_tuning optimize --simulate --config examples/simulation-speed.json --run-dir runs\speed-sim-001 --export-best runs\speed-best-unverified.json
& $python -m gimbal_tuning replay runs\speed-sim-001 --output runs\speed-replay.json
```

Direct integrations are pinned: `pyserial==3.5`, `bayesian-optimization==3.1.0`,
`packaging==25.0`. `packaging` is explicitly required because bayes_opt 3.1.0 imports
it but omits it from its distribution dependencies. Build dependency is
`setuptools==80.9.0`. `pip install .` alone installs the stdlib foundation; optional
modules are imported only when used. Transitive numerical packages are not a full
platform lock; actual installed versions are recorded in run metadata when available.

**Only after the user has independently filled/confirmed the hardware configuration,
firmware deployment, protection, units, fixed pose and relevant gate:**

```powershell
& $python -m gimbal_tuning validate my-hardware.json --gate B --hardware
& $python -m gimbal_tuning status --port COM5
& $python -m gimbal_tuning baseline --port COM5 --authorize-motion --config my-hardware.json --run-dir runs\baseline-hw-001
& $python -m gimbal_tuning trial --port COM5 --authorize-motion --config my-hardware.json --run-dir runs\trial-hw-001
# Separately accept measured baseline/performance/search/budget before Gate C:
& $python -m gimbal_tuning validate my-hardware.json --gate C --hardware
& $python -m gimbal_tuning optimize --port COM5 --authorize-motion --config my-hardware.json --run-dir runs\search-hw-001 --export-best runs\candidate-unverified.json
```

These are usage instructions, not evidence of hardware tests. `--simulate` and
`--port` are mutually exclusive; every experiment requires one explicitly. Hardware
motion additionally requires `--authorize-motion`. `status` sends HELLO then GET_STATUS;
HELLO cancels any existing authorization/trial and is **not passive attachment**.
No port is selected implicitly. VOFA must not own the device serial port.

Python/Windows threads are not a hard real-time safety controller. Device heartbeat,
feedback, task/timing and bounded CAN protections remain authoritative; serial failure
or a STOP without confirmation requires the independent physical e-stop/monitor.
Before movement, `clock_rng_verified=true` requires onsite verification of actual
system/PLLQ/RNG/TIM5 clocks and exact-chip clock-domain legality. The source HSE12MHz,
PLLM12/PLLN360/PLLP2/PLLQ7 arithmetic gives PLLQ approximately 51.43MHz, not a validated
nominal 48MHz RNG domain. A nonzero RNG value or DRDY alone does not prove compliance.
Do not blindly change the global PLL; real clocks/chip legality remain unverified,
not a software-test pass or confirmed runtime root cause.
A confirmed zero protocol status is not proof of actuator torque or physical stopping.
There is no new global watchdog reset or whole-arm torque cut in this host package.

## Gates, algorithms and versions

Strict JSON rejects duplicate keys, NaN/Infinity/exponent overflow, unknown/missing
fields and bool where numeric values are required. Wire binary32 representation and
arithmetic/timer guards are checked; they do **not** establish physical limits.

* Gate A: schema/identity-read access, no motion approval. The null/unconfirmed
  hardware template intentionally passes only Gate A.
* Gate B: every safety, trajectory, PID and identity value must be explicit. Human
  supervision, support/e-stop, coast margin, fixed pose, units/direction/zero and
  initialization/small-trajectory confirmations are required. A zero-step may be
  acquired diagnostically, but is not an excited/scored BO experiment.
* Gate C adds separately accepted baseline/preparation/performance/search/budget,
  seed/scoring definition/units/firmware and **bidirectional** nonzero scenarios.
  Excitation must remain nonzero after binary32 conversion. The full trial-count
  product must fit positive u32; schedules are iterated, not prebuilt.

The simulation files contain invented numbers and synthetic-only acceptances. They
are rejected by hardware gates. Never flip their marker and treat their numbers or
confirmations as measured hardware evidence. The hardware template supplies no guessed
hardware values and no preaccepted baseline.

Algorithms: 0 legacy discrete PID/clamped I (default), 1 conditional integration v1.
Hardware algorithm 1 Gate B requires `conditional_trial_authorized=true`, an explicit
human opt-in to one protected diagnostic acquisition. It does **not** require prior
`conditional_baseline_accepted`: the baseline being measured is not already accepted.
Gate C algorithm 1 additionally requires `conditional_baseline_accepted=true` for that
exact independently tested baseline, alongside normal `gate_c.baseline_accepted`.
Legacy algorithm 0 may leave both conditional flags false. No diagnostic sets either
acceptance automatically. Repository gains `1.8,25,800,16` are an
**unaccepted original baseline**, not an approved seed or inferred improvement target.
PID I/D remain per sample, nominally 1 ms; angles rad, speeds rad/s, feedback current A,
temperature degC, output group-total ESC commands split 50/50.

Gate C `stage`:

* `speed_pi`: mode-1 speed-reference scenarios only; position P/D fixed exactly to
  the baseline, only speed P/I search axes vary.
* `position_pd`: mode-0 position scenarios only; `speed_loop_accepted=true` explicitly
  binds the fixed speed P/I in the configured baseline to a separately accepted inner
  loop. Only position P/D vary. A speed-search winner is **not** automatically accepted.

Search axes must have nonempty inward-rounded binary32 intervals. Inactive gain ranges
are fixed points equal to the accepted baseline. Effective gains are rounded and checked
before staging; BO registration and exports use the actual effective binary32 values.

Every explicit new run does HELLO + INFO/GET_CONFIG inspection and allocates baseline/
candidate transaction versions above the observed active/stored baseline versions.
Same-device second runs do not assume versions reset to 1/2. Acceptance binds exact
**gains + algorithm + configuration/pose**, not the rebased wire transaction number.
Logs preserve declared baseline, effective baseline/version, all stage versions and the
configuration hash. Reauthorization is a fresh explicit run, never automatic recovery.

## Execution, liveness and stop semantics

`ExperimentManager` uses one `IOOwner` thread for all RX/parser work, sequence allocation
and serialized writes. Heartbeats and host wall-budget supervision run there, independent
of the optimizer, scorer, CLI and fsync writer. Serial read/write timeout is at most
heartbeat_timeout/8; normal service cadence is bounded. A missed host scheduling
limit, missing ACK, boot/session change, protection, preparation failure, malformed/
nonfinite telemetry or excessive data loss stops the entire batch. Nothing retries START,
automatically reconnects, clears faults or chooses a rescue candidate. Independent MCU
watchdog behavior is still needed if Python/the OS stalls completely.

Normal completion requires the full RESULT, the immediate next COMPLETE_ZERO telemetry,
and a subsequent clean GET_STATUS (correct trial/parameter version and authorized batch).
A CAN fault after RESULT therefore cannot be hidden by the earlier result alone.
Only after those confirmations may an explicitly authorized batch stage its next trial.
The simulator executes transient APPLY_CANDIDATE in the same sampled tick that completes
the continuously-at-rest SETTLE hold, matching the portable core's reset/seed policy.
Losing rest resets the hold; no extra tick may invalidate a previously checked snapshot.
Normal search end sends END_BATCH and queries healthy zero/revoked status. Count-budget
completion is reported as `BUDGET_EXHAUSTED/termination=candidate_count`. A wall-budget
stop, Ctrl+C, transport/telemetry/preparation/protection failure sends STOP once and awaits
its zero/revocation ACK; missing confirmation sets `requires_manual_estop=true`. STOP
invalidates the session, so a wall-budget/error stop cannot subsequently END_BATCH in that
session. There is no automatic retry or continuation after it.

The total count includes the initial baseline candidate. **All** configured scenarios
and repeats are run in the same order for each candidate. Baseline evaluation/registration
precedes any `suggest()`. Full performance-infeasible observations may continue; partial,
protection, communication or telemetry-invalid candidates never become ordinary BO data.
Main-thread work may exhaust the approved wall budget even while the group is zero;
the owner stops independently rather than granting an optimizer an unbounded extension.

## Scoring definition: `constraints_first_v2`

Performance schema has separate position and speed thresholds:
`max_overshoot_rad`, `final_error_rad`, `sustained_oscillation_limit_rad`;
`max_overshoot_rad_s`, `final_error_rad_s`, `sustained_oscillation_limit_rad_s`;
plus explicit `hold_ms`, `max_settling_ms`, `max_telemetry_gap_ms`, `max_missing_fraction`.
No rad threshold is reused as rad/s.

Before scoring: verify boot/session/trial/version, finite rows, monotonic unwrapped sample/
time counters, normal RESULT boundaries/duration, zero final command, next zero sample
and clean queried status. All nonterminal RUN rows must remain motion-enabled; only
the terminal RUN row identified by RESULT's exact sample/time endpoints must be disabled
with output and both commands zero, retaining RUN state/flag and candidate version.
An arbitrary disabled mid-RUN row, STOP/partial window, or enabled endpoint is invalid.
Duplicate/reordered/reset counters, missing endpoints, faults,
partial duration or excessive gaps/loss invalidate the trial. Loss is conservatively
estimated from cadence gaps with the configured control-gap allowance and cumulative TX
loss; any RX drop increase invalidates. This is an inferred transport-loss measure, not
proof that no sub-cadence sample was ever lost. Raw evidence is always retained.

Metrics use motor0 closed-loop position/speed; motor-pair/feedback/other-joint safety
remains a separate device protection gate. The explicit final trial target is the
reference, including speed tests that initially ramp from zero. Stability cannot start
before the speed reference reaches that final value. Settling is the beginning of the
final continuously in-band suffix lasting at least hold_ms, **not first crossing**.
Constraints, in the mode's units: directional maximum overshoot, final absolute error,
maximum settling time, and peak-to-peak tail motion (last max(hold,duration/4), after
final reference readiness). Tail span is an explicit conservative constraint, not a
claim to identify a physical oscillation mechanism.

For complete feasible trials:

```text
cost = (settling_time/duration + IAE/(excitation*duration)
        + sum(abs(delta(command0))+abs(delta(command1)))/(max_output*(N-1))) / 3
utility = 1/(1+cost)                         # > 0
```

Output variation sums each motor's absolute command deltas, so opposite ESC direction
signs cannot cancel activity; the both-positive 50/50 variant is unchanged.
IAE integrates absolute error to the **final** reference throughout RUN, so the fixed
speed ramp's common reference history is preserved across candidates. Infeasible full
trials get `-1 - violation/(1+violation)`, with violation the summed positive normalized
constraint excesses: every infeasible result ranks below every feasible one. Candidate
utility is the **minimum over every repeat and scenario**, never a mean that conceals
one failure. Nonfinite normalized ratios/costs/utilities (including overflow from tiny
but finite thresholds) invalidate scoring before registration; aggregate utility/statistics
must also be finite. No fabricated feasible or NaN observation is accepted.
Per-scenario descriptive standard deviations are reported; fewer than two
repeats is explicitly insufficient, and no confidence/physical repeatability claim is
made even when simulated repeats agree.

Best export is always `publishable=false`, `requires_independent_validation=true`, and
`accepted_speed_loop=false`. It includes actual effective parameters, feasibility,
uncertainty, configuration provenance and trial trace IDs/records into the raw run
manifest. If every complete candidate is infeasible, the least-bad one remains explicitly
infeasible and unverified. No export causes device loading or movement. Independent
repetition and human review are required before release/inner-loop acceptance.

## Durable records, VOFA and offline analysis

Each new run directory contains:

* `config.json` exact parsed snapshot and `config.original.json` exact original bytes
  when loaded from a file; `meta.json` includes source/config hash, algorithm, identity/
  units, score definition, stage, seed/budget and validation markers.
* `raw.jsonl`: authoritative ordered TX-attempt/RX chunks as hex with monotonic PC times.
  A TX attempt is not proof the device received/applied it. `frames.jsonl` records decoded
  wire identities/sequence/payload, including malformed semantic evidence.
* `telemetry.jsonl`: decoded rows, trial/parameter versions and frame indexes. Nonfinite
  values are preserved as strings alongside raw bytes and invalidation events, never
  converted into a successful numeric score.
* `results.jsonl`, `observations.jsonl`, `events.jsonl`, `summary.json`, and a final SHA256
  file manifest. Trial/observation barriers fsync; file hashes detect accidental changes,
  not a malicious writer with permission to rewrite the manifest.

A bounded asynchronous log writer isolates storage latency from heartbeat service.
It periodically flushes/fsyncs at a nominal 250ms interval in addition to explicit
trial/observation barriers; OS/power-loss durability is not a real-time guarantee.
Queue/storage failure is fatal, not silent record loss. Emergency STOP/ACK processing
still proceeds if logging fails. An absent manifest means incomplete/unfinalized evidence;
replay refuses it rather than fabricating a successful run.

Optional `--vofa-port 1347 --vofa-host 127.0.0.1` forwards read-only nonblocking UDP
JustFloat (little-endian channels + `00 00 80 7F`). Only numeric loopback endpoints are
allowed; UDP drops/disconnection cannot affect authoritative logging or device protection.
Channels: target_pos, pos0, pos1, target_vel, vel0, vel1, pos_error, speed_error, integral,
output, command0, command1, state, fault, active_version. Actual VOFA compatibility remains
an external runtime acceptance item; no VOFA process was verified by these tests.

```powershell
& $python -m gimbal_tuning analyze runs\speed-sim-001 --output runs\analysis.json
& $python -m gimbal_tuning replay runs\speed-sim-001 --propose --output runs\offline-proposal.json
& $python -m gimbal_tuning compare runs\legacy-001 runs\conditional-001 --output runs\diagnostics.json
```

Replay verifies the manifest/config provenance, decodes authoritative RX again, checks
effective gains/baseline/trial against raw TX transactions and recomputes valid scores/
aggregates. Optional `--propose` reconstructs observations for one **offline** suggestion.
It cannot open a port or resume a motion batch. `compare` requires legacy then conditional
recordings, reports configuration comparability and unscored descriptive diagnostics
(integral/output/error peaks, saturation fraction, feedback age, observed control dt,
final-reference error/overshoot), and never establishes
causal control improvement or automatically accepts a baseline.

## APIs and validation boundary

* `protocol`: `Frame`, `StreamDecoder`, `Safety`, `Parameters`, `Trial`, `Telemetry`,
  `Status`, `decode_result`, `CounterUnwrapper`; wire layouts unchanged (140/24/124 bytes).
* `Configuration.load(...).validate('A'|'B'|'C', hardware=...)`; new exact local schema
  fields described above, score version v2. Old v1 score configs must not silently migrate.
* `ExperimentManager(config, run_dir, hardware=False, port=None, authorize_motion=False,
  transport_factory=None, vofa=None).run_single(baseline=False)` / `.run_search(proposer=None)`.
  Instances are single-use. Pre-admission errors raise before opening; operational failures
  return classified summaries with STOP/zero confirmation and durable evidence.
* `BayesianProposer.register(effective_parameters, aggregate)` / `.suggest()`;
  `materialize` enforces stage axes/rounded limits. No library `maximize()` loop.
* `analysis.replay(run_dir, propose=False)` / `compare_baselines(...)` are offline-only.
* Legacy `ProtocolClient` remains useful for protocol tests/read queries. Actual
  `SerialTransport` hardware capability cannot be disabled by omitting/clearing the
  client flag. Hardware payload admission permits version-only rebasing of the exact
  approved binary32 baseline/candidate, not gain/algorithm mutation. Use the manager,
  not a blocking ad hoc client loop, for hardware experiments/independent heartbeats.

Tests cover protocol corruption/replay/barriers; two explicit sessions on the same device;
config/stage/binary32/u32 boundaries; slow optimizer/fsync heartbeat continuity and owner
budget watchdog; uncertain START/STOP loss/disconnect/reboot; late post-RESULT fault;
dropped/reordered/nonfinite telemetry; scoring endpoints/continuous hold/ramp/units;
mocked UDP disconnection; durable replay/hash checks; and real pinned BO propose/register/
seed and simulated search smoke tests in an isolated optional environment.

Not verified here: Windows execution, Keil compilation/flash, real serial timing,
MCU boot entropy/control jitter, actual CAN/torque zero, feedback scaling/physical PID
stability, mechanical support/e-stop or VOFA runtime. No physical baseline acceptance,
anti-windup improvement or hardware BO result is claimed.
