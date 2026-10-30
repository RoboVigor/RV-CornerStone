# Gimbal tuning wire contract v1

Software implementation contract for `gimbal-pid-implementation-plan.md`. Little endian; IEEE-754 binary32; angles rad, velocities rad/s, current feedback A, temperature °C. Output is **ESC command units**, not A. PID I/D are per control sample, nominally 1 ms. Hardware values are never inferred from the examples.

## Framing and replay rules

`A5 5A | version:u8=1 | type:u8 | payload_length:u16 | boot_id:u32 | session:u32 | seq:u32 | payload | crc:u16`.
Header including sync = 18 bytes, CRC = CRC-16/CCITT-FALSE (init FFFF, polynomial 1021, no reflection/xorout) over bytes from version through payload. Maximum payload 256 bytes (frame 276). No C struct is transmitted. Invalid CRC/version/length is discarded with byte-wise resynchronization; incomplete frame expires after 100 ms. Receivers bound buffering and count rejected/dropped data.

`HELLO` has boot/session/seq all zero and no payload. It **revokes any previous batch**, stops an active trial and allocates a new device-owned, monotonically increasing session for this boot. It does not clear a latched fault. INFO returns the new boot/session. Firmware boot ID must be nonzero and change on reboot; initialization fails closed if entropy is unavailable. Replayed HELLO can stop, never re-enable. Ordinary commands must match boot/session and have nonzero strictly increasing seq (no wrap in a session). Only an exact duplicate of the most recently accepted command is ACKed without side effects. A duplicate HEARTBEAT never refreshes liveness. Conflicting duplicates and older/out-of-order seq are rejected. There is no automatic retry of motion commands; an uncertain ACK requires status inspection and a safe stop, not another START.

`STOP` with valid framing/version and empty payload is accepted irrespective of session/sequence, zeros the group, cancels the batch and invalidates the session. It cannot clear a fault. Another HELLO is required before any new authorization. Other mutations only occur in the control task at a sample boundary, not in the UART ISR. Read queries cannot grant motion. Reconnect never resumes a batch.

## Message IDs and payloads

| Type | ID | Payload |
|---|---:|---|
| HELLO | 1 | empty, special header above |
| GET_STATUS | 2 | empty |
| GET_CONFIG | 3 | empty |
| SET_SAFETY_CONFIG | 10 | Safety below, only zero-output states |
| SET_BASELINE | 11 | Parameters below, only zero-output, outside batch |
| STAGE_CANDIDATE | 12 | Parameters below, only zero-output |
| START_BATCH | 13 | batch_id:u32, max_trials:u32 (both nonzero) |
| START_TRIAL | 14 | trial_id:u32, mode:u32, origin:f32, target:f32, duration_ms:u32 |
| HEARTBEAT | 15 | empty |
| STOP | 16 | empty |
| CLEAR_FAULT | 17 | empty; requires configured, healthy fresh feedback and heartbeat |
| END_BATCH | 18 | empty; only zero-output, cancels authorization and clears current batch_id to 0 |
| ACK | 128 | command_type:u32, error:u32=0, Status |
| INFO | 129 | 8 u32: protocol=1, firmware=65536, capabilities=3, motor0_id=0x205, motor1_id=0x206, baud=500000, nominal_control_us=1000, telemetry_size=124; then Status |
| STATUS | 130 | Status |
| CONFIG | 131 | safety_valid:u32, Safety (zero if absent), baseline_valid:u32, Parameters (zero if absent) |
| TELEMETRY | 144 | Telemetry below |
| RESULT | 145 | 7 u32: trial_id, classification, fault, run_start_seq, run_end_seq, run_start_us, run_end_us; then Status |
| FAULT | 146 | Status |
| NACK | 255 | command_type:u32, error:u32, Status |

Responses to commands echo their seq; asynchronous messages use seq=0. All replies carry current boot/session (STOP ACK carries session=0). CONFIG is 172 bytes, ACK/NACK 56, INFO 80, STATUS/FAULT 48, RESULT 76.

### Status: 12 u32 (48 bytes)

In order: `state, fault, active_version, applied_seq, sample_seq, trial_id, batch_id, flags, tick_us, tx_dropped, rx_dropped, last_command_seq`.
Flags: bit0 safety configured, bit1 baseline configured, bit2 candidate staged, bit3 batch authorized, bit4 motion enabled.

States: 0 IDLE_ZERO, 1 PRECHECK, 2 PREPARE_BASELINE, 3 SETTLE, 4 APPLY_CANDIDATE, 5 RUN, 6 COMPLETE_ZERO, 7 STOPPED_ZERO, 8 FAULT_LATCHED_ZERO.
Faults: 0 NONE, 1 HEARTBEAT, 2 FEEDBACK, 3 POSITION, 4 SPEED, 5 CURRENT, 6 TEMPERATURE, 7 PAIR_MISMATCH, 8 OTHER_JOINT, 9 PREPARE_TIMEOUT, 10 SETTLE_TIMEOUT, 11 CONTROL_TIMING, 12 BATCH_TIMEOUT, 13 NONFINITE, 14 TRANSPORT.
NACK errors: 1 BAD_SESSION, 2 REPLAY, 3 BAD_LENGTH, 4 BAD_STATE, 5 INVALID_CONFIG, 6 INVALID_PARAMS, 7 NOT_READY, 8 INVALID_TRIAL, 9 BUDGET, 10 UNKNOWN_COMMAND.
Result classifications: 0 COMPLETE, 1 STOPPED, 2 PROTECTION, 3 PREPARATION_FAILED. Host adds COMMUNICATION_INVALID, TELEMETRY_INVALID and BUDGET_EXHAUSTED; these never become ordinary BO observations.

### Safety: 26 f32 then 9 u32 (140 bytes)

Float order:
`pos_min, pos_max, margin, max_speed, max_output, max_current, max_temperature, max_pair_error, prepare_speed, prepare_accel, settle_pos_tol, settle_speed_tol, other_pos_tol, other_speed_tol, other_target_1, other_target_2, other_target_3, gain_min_pos_p, gain_min_pos_d, gain_min_speed_p, gain_min_speed_i, gain_max_pos_p, gain_max_pos_d, gain_max_speed_p, gain_max_speed_i, max_integral`.

Integer order:
`feedback_timeout_ms, heartbeat_timeout_ms, prepare_timeout_ms, settle_timeout_ms, settle_hold_ms, max_trial_ms, telemetry_period_ms, batch_timeout_ms, max_control_gap_us`.

All values finite and required. Positive limits/tolerances; nonnegative margin; strict nonempty interior `[pos_min+margin,pos_max-margin]`; prepare_speed <= max_speed; gain ranges finite/nonnegative and min <= max; max_output <= 32767 (group total, split 50/50); max_integral <= max_output. All float magnitudes <= 1e6 (arithmetic representation guard), max_integral > 0. All millisecond values nonzero and <= 1,800,000 (32-bit microsecond deadline representation guard); feedback timeout <= 60000 ms; heartbeat <= 60000 ms; telemetry period >= 5 ms; hold <= settle timeout; trial <= batch timeout; nominal control period 1000 us <= max_control_gap_us <= 100000 us. These are representation/transport constraints, **not verified physical safety limits**. Runtime limits may not be changed during an authorized batch. Other joints retain their original targets/control; their measured positions/speeds, feedback ages and actual command targets must match the configured fixed pose within tolerances.

### Parameters: u32 + 4 f32 + u32 (24 bytes)

`version, pos_p, pos_d, speed_p, speed_i, algorithm`.
Algorithm 0 = legacy discrete PID with clamped I; 1 = conditional integration v1, joint0 only. Position I=0, speed D=0. Limits come from Safety. Version must be nonzero and monotonically increasing for newly staged sets in a session (HELLO resets the staging barrier to max(active_version, baseline.version)); candidate algorithm equals baseline algorithm during a batch. No Flash writes, implicit best loading or global PID semantic changes. Staging is not application; actual version/applied sample are reported in telemetry/status. PREPARE uses baseline. At APPLY, I resets to zero and derivative history is seeded with the measured pre-step error; this deterministic reset-at-rest policy is recorded, not advertised as physical bumplessness. RUN target step retains original discrete D behavior. Optional conditional integration requires a separately accepted algorithm baseline before hardware BO.

### Trial and batch behavior

Mode 0 = position step (`target` rad); mode 1 = speed reference (`target` rad/s, outer loop bypassed, position bounds still supervised). Origin is always rad. Origin/position target are inside the margin; speed magnitude <= max_speed. Duration is in `(0,max_trial_ms]`. START_TRIAL requires a live explicit batch and a staged candidate. Trial IDs strictly increase within a batch. START_BATCH requires safety, baseline, staged parameters and a fresh heartbeat; no motion until START_TRIAL. Batch count and wall time are enforced by MCU and host. No new batch while one is active.

PRECHECK verifies all feedback/other joints, then PREPARE applies baseline with reset histories and a bounded-velocity/acceleration reference from actual position to origin. SETTLE requires both motors at rest/near origin for settle_hold_ms, with its own deadline. APPLY uses staged parameters as one transaction. RUN emits its first and last samples even off the normal telemetry cadence. At normal duration end, the final RUN endpoint already records zero output with motion-enabled clear (RUN-window flag remains set), precedes RESULT, and the immediately next sample is COMPLETE_ZERO with both outputs zero. Earlier RUN samples retain motion-enabled even when their numeric command happens to be zero. Each next trial re-reads actual position. Candidate stage is consumed; subsequent trial needs explicit staging. STOP/any fault cancel the batch. Only COMPLETE_ZERO inside a still authorized batch permits the next automatic trial. CLEAR_FAULT verifies recovered conditions but never enables motion. No protection auto-fallback or auto-retry.

### Telemetry: 15 u32 then 16 f32 (124 bytes)

U32 order: `tick_us, sample_seq, trial_id, state, fault, active_version, applied_seq, tx_dropped, rx_dropped, control_dt_us, feedback_age0_ms, feedback_age1_ms, feedback_count0, feedback_count1, flags`.
Float order: `target_pos, target_vel, pos0, pos1, vel0, vel1, current0, current1, temperature0, temperature1, pos_error, speed_error, integral, output, command0, command1`.
Flags bit0 saturated (position/speed output or speed-I limit), bit1 motion enabled, bit2 baseline active, bit3 RUN window. Both timestamps and sample counters wrap u32; host unwraps using monotonic deltas and rejects stale/reordered samples. Drop counters are cumulative; raw frames and decoded rows are recorded independent of UDP. MCU sample/control dt and feedback counters expose actual rather than assumed timing. Only complete RUN windows matching boot/session/trial/parameter version and RESULT boundaries may be scored. Missing endpoints, excessive telemetry loss/gaps or faults invalidate a trial. Telemetry period lower bound budgets <=28.8 kB/s at 500000 baud before command overhead.

VOFA forwarding: UDP to a configured loopback endpoint, little-endian float channels followed by `00 00 80 7F` (JustFloat). It is read-only, best effort, and cannot affect device protection or authoritative serial logging. No device serial reconnection, batch restoration, resume or default-parameter publication is automatic.
