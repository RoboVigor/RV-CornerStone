#ifndef DRIVER_GIMBAL_TUNING_H
#define DRIVER_GIMBAL_TUNING_H

/* Portable, allocation-free joint-0 tuning core. See docs/gimbal-tuning-protocol.md.
 * Call Feed/Tick only from the control task. ISR code only copies bytes/feedback.
 * The hardware adapter must enforce the returned zero at the final CAN slots. */
#include <stdint.h>
#include <stddef.h>

#define GT_PROTOCOL 1u
#define GT_MAX_PAYLOAD 256u
#define GT_MAX_FRAME 276u
#define GT_SAFETY_SIZE 140u
#define GT_PARAM_SIZE 24u
#define GT_STATUS_SIZE 48u
#define GT_TELEMETRY_SIZE 124u
#define GT_FRAME_TIMEOUT_US 100000u
#define GT_MAX_DURATION_MS 1800000u

enum GT_Message {
    GT_HELLO=1, GT_GET_STATUS=2, GT_GET_CONFIG=3,
    GT_SET_SAFETY=10, GT_SET_BASELINE=11, GT_STAGE=12, GT_START_BATCH=13,
    GT_START_TRIAL=14, GT_HEARTBEAT=15, GT_STOP=16, GT_CLEAR_FAULT=17,
    GT_END_BATCH=18, GT_ACK=128, GT_INFO=129, GT_STATUS=130, GT_CONFIG=131,
    GT_TELEMETRY=144, GT_RESULT=145, GT_FAULT=146, GT_NACK=255
};
enum GT_State {
    GT_IDLE_ZERO, GT_PRECHECK, GT_PREPARE_BASELINE, GT_SETTLE,
    GT_APPLY_CANDIDATE, GT_RUN, GT_COMPLETE_ZERO, GT_STOPPED_ZERO,
    GT_FAULT_LATCHED_ZERO
};
enum GT_Fault {
    GT_F_NONE, GT_F_HEARTBEAT, GT_F_FEEDBACK, GT_F_POSITION, GT_F_SPEED,
    GT_F_CURRENT, GT_F_TEMPERATURE, GT_F_PAIR_MISMATCH, GT_F_OTHER_JOINT,
    GT_F_PREPARE_TIMEOUT, GT_F_SETTLE_TIMEOUT, GT_F_CONTROL_TIMING,
    GT_F_BATCH_TIMEOUT, GT_F_NONFINITE, GT_F_TRANSPORT
};
enum GT_Error {
    GT_E_OK, GT_E_SESSION, GT_E_REPLAY, GT_E_LENGTH, GT_E_STATE,
    GT_E_CONFIG, GT_E_PARAMS, GT_E_READY, GT_E_TRIAL, GT_E_BUDGET, GT_E_COMMAND
};
enum GT_ResultClass { GT_R_COMPLETE, GT_R_STOPPED, GT_R_PROTECTION, GT_R_PREPARATION_FAILED };

typedef struct {
    float pos_min, pos_max, margin, max_speed, max_output, max_current;
    float max_temperature, max_pair_error, prepare_speed, prepare_accel;
    float settle_pos_tol, settle_speed_tol, other_pos_tol, other_speed_tol;
    float other_target[3];
    float gain_min[4], gain_max[4]; /* pos P, pos D, speed P, speed I */
    float max_integral;
    uint32_t feedback_timeout_ms, heartbeat_timeout_ms, prepare_timeout_ms;
    uint32_t settle_timeout_ms, settle_hold_ms, max_trial_ms;
    uint32_t telemetry_period_ms, batch_timeout_ms, max_control_gap_us;
} GT_Safety;

typedef struct {
    uint32_t version;
    float pos_p, pos_d, speed_p, speed_i;
    uint32_t algorithm; /* 0 legacy, 1 conditional-I v1. Only joint0. */
} GT_Parameters;

typedef struct {
    float pos[2], vel[2], current[2], temperature[2];
    uint32_t age_ms[2], feedback_count[2];
    uint8_t online[2];
    int8_t direction[2]; /* ESC sign; positions/velocities are already joint coordinates. */
    float other_pos[3], other_vel[3], other_command[3];
    uint32_t other_age_ms[3];
    uint8_t other_online[3];
    uint8_t external_stop;
} GT_Feedback;

typedef struct {
    float last_error, integral, output;
} GT_PIDState;

/* Return 1 after copying a frame into owned bounded storage, 0 if full.
 * Never block, transmit synchronously, or retain the pointer. */
typedef int (*GT_Send)(void *user, const uint8_t *frame, uint16_t length);

typedef struct {
    uint32_t boot_id, session, session_counter, last_seq;
    uint32_t state, fault, now_us, last_tick_us, sample_seq, dt_us;
    uint32_t active_version, applied_seq, highest_version;
    uint32_t batch_id, max_trials, trials_started, last_trial_id, trial_id;
    uint32_t batch_at_us, heartbeat_at_us, state_at_us, settle_at_us;
    uint32_t run_at_us, run_start_seq, run_end_seq, run_end_us;
    uint32_t trial_mode, trial_duration_ms;
    uint32_t tx_dropped, rx_dropped, last_telemetry_us;
    uint8_t safety_valid, baseline_valid, staged_valid, batch_active;
    uint8_t heartbeat_valid, settle_tracking, motion_enabled, saturated;
    uint8_t baseline_active, have_tick, complete_pending;
    float origin, target, prepare_origin, prepare_distance, prepare_sign;
    float prepare_accel_time, prepare_coast_time, prepare_peak_speed;
    float target_pos, target_vel, pos_error, speed_error, output;
    float command[2];
    GT_Safety safety;
    GT_Parameters baseline, staged, active;
    GT_PIDState pos_pid, speed_pid;
    GT_Feedback feedback;
    GT_Send send;
    void *send_user;
    uint8_t rx[GT_MAX_FRAME], last_request[GT_MAX_FRAME], last_reply[GT_MAX_FRAME];
    uint16_t rx_len, last_request_len, last_reply_len;
    uint32_t rx_at_us;
} GT_Context;

uint16_t GT_CRC16(const uint8_t *data, size_t length);
uint32_t GT_ReadU32(const uint8_t *p);
float GT_ReadFloat(const uint8_t *p);
void GT_WriteU32(uint8_t *p, uint32_t v);
void GT_WriteFloat(uint8_t *p, float v);
uint16_t GT_Encode(uint8_t *out, uint8_t type, uint32_t boot,
                   uint32_t session, uint32_t seq, const uint8_t *payload, uint16_t len);
void GT_EncodeSafety(uint8_t *p, const GT_Safety *s);
void GT_DecodeSafety(GT_Safety *s, const uint8_t *p);
void GT_EncodeParams(uint8_t *p, const GT_Parameters *v);
void GT_DecodeParams(GT_Parameters *v, const uint8_t *p);
int GT_ValidateSafety(const GT_Safety *s);
int GT_ValidateParams(const GT_Safety *s, const GT_Parameters *v);
void GT_Init(GT_Context *c, uint32_t boot_id, GT_Send send, void *user);
void GT_Feed(GT_Context *c, uint8_t byte, uint32_t now_us);
void GT_Tick(GT_Context *c, uint32_t now_us, const GT_Feedback *feedback);
void GT_TransportFault(GT_Context *c);
int GT_MotionAllowed(const GT_Context *c);
/* Final wire-slot rule: preserve every unrelated bus/ID/slot and force the
 * joint0 current frame even when the online motor mask is empty. */
int16_t GT_CANSlot(const GT_Context *c, uint8_t bus, uint16_t send_id,
                   uint16_t motor_id, int16_t original, uint8_t *force_frame);
/* Pure discrete control helpers also used by host tests. */
float GT_PIDStep(GT_PIDState *s, float error, float p, float i, float d,
                 float max_output, float max_integral, uint32_t algorithm);
void GT_ResetControl(GT_Context *c, float position_reference, float speed_reference);

#endif
