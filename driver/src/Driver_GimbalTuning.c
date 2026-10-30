#include "Driver_GimbalTuning.h"
#include <string.h>
#include <math.h>
#include <float.h>

typedef char GT_Binary32Required[(sizeof(float)==4 && FLT_RADIX==2 && FLT_MANT_DIG==24 && FLT_MAX_EXP==128)?1:-1];

/* Explicit serialization: neither wire order nor validation depends on padding. */
uint32_t GT_ReadU32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
void GT_WriteU32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24);
}
float GT_ReadFloat(const uint8_t *p) { uint32_t v=GT_ReadU32(p); float f; memcpy(&f,&v,4); return f; }
void GT_WriteFloat(uint8_t *p, float f) { uint32_t v; memcpy(&v,&f,4); GT_WriteU32(p,v); }
uint16_t GT_CRC16(const uint8_t *p, size_t len) {
    uint16_t crc=0xffffu; size_t n; unsigned i;
    for(n=0;n<len;n++) {
        crc^=(uint16_t)p[n]<<8;
        for(i=0;i<8;i++) crc=(uint16_t)((crc&0x8000u)?((uint32_t)crc<<1)^0x1021u:(uint32_t)crc<<1);
    }
    return crc;
}
uint16_t GT_Encode(uint8_t *out, uint8_t type, uint32_t boot, uint32_t session,
                   uint32_t seq, const uint8_t *payload, uint16_t len) {
    uint16_t crc;
    if(len>GT_MAX_PAYLOAD) return 0;
    out[0]=0xa5; out[1]=0x5a; out[2]=GT_PROTOCOL; out[3]=type;
    out[4]=(uint8_t)len; out[5]=(uint8_t)(len>>8);
    GT_WriteU32(out+6,boot); GT_WriteU32(out+10,session); GT_WriteU32(out+14,seq);
    if(len) memcpy(out+18,payload,len);
    crc=GT_CRC16(out+2,(size_t)len+16);
    out[len+18]=(uint8_t)crc; out[len+19]=(uint8_t)(crc>>8);
    return (uint16_t)(len+20);
}

#define SAFETY_FLOATS(X) \
 X(pos_min) X(pos_max) X(margin) X(max_speed) X(max_output) X(max_current) \
 X(max_temperature) X(max_pair_error) X(prepare_speed) X(prepare_accel) \
 X(settle_pos_tol) X(settle_speed_tol) X(other_pos_tol) X(other_speed_tol)
#define SAFETY_INTS(X) \
 X(feedback_timeout_ms) X(heartbeat_timeout_ms) X(prepare_timeout_ms) \
 X(settle_timeout_ms) X(settle_hold_ms) X(max_trial_ms) X(telemetry_period_ms) \
 X(batch_timeout_ms) X(max_control_gap_us)
void GT_EncodeSafety(uint8_t *p, const GT_Safety *s) {
    unsigned n=0,i;
#define EF(k) GT_WriteFloat(p+4*n++,s->k);
    SAFETY_FLOATS(EF)
#undef EF
    for(i=0;i<3;i++) GT_WriteFloat(p+4*n++,s->other_target[i]);
    for(i=0;i<4;i++) GT_WriteFloat(p+4*n++,s->gain_min[i]);
    for(i=0;i<4;i++) GT_WriteFloat(p+4*n++,s->gain_max[i]);
    GT_WriteFloat(p+4*n++,s->max_integral);
#define EU(k) GT_WriteU32(p+4*n++,s->k);
    SAFETY_INTS(EU)
#undef EU
}
void GT_DecodeSafety(GT_Safety *s, const uint8_t *p) {
    unsigned n=0,i;
#define DF(k) s->k=GT_ReadFloat(p+4*n++);
    SAFETY_FLOATS(DF)
#undef DF
    for(i=0;i<3;i++) s->other_target[i]=GT_ReadFloat(p+4*n++);
    for(i=0;i<4;i++) s->gain_min[i]=GT_ReadFloat(p+4*n++);
    for(i=0;i<4;i++) s->gain_max[i]=GT_ReadFloat(p+4*n++);
    s->max_integral=GT_ReadFloat(p+4*n++);
#define DU(k) s->k=GT_ReadU32(p+4*n++);
    SAFETY_INTS(DU)
#undef DU
}
void GT_EncodeParams(uint8_t *p, const GT_Parameters *v) {
    GT_WriteU32(p,v->version); GT_WriteFloat(p+4,v->pos_p); GT_WriteFloat(p+8,v->pos_d);
    GT_WriteFloat(p+12,v->speed_p); GT_WriteFloat(p+16,v->speed_i); GT_WriteU32(p+20,v->algorithm);
}
void GT_DecodeParams(GT_Parameters *v, const uint8_t *p) {
    v->version=GT_ReadU32(p); v->pos_p=GT_ReadFloat(p+4); v->pos_d=GT_ReadFloat(p+8);
    v->speed_p=GT_ReadFloat(p+12); v->speed_i=GT_ReadFloat(p+16); v->algorithm=GT_ReadU32(p+20);
}
static int finite_float(float f) { return f==f && f<=FLT_MAX && f>=-FLT_MAX; }
static float clamp(float f,float lo,float hi) { return f<lo?lo:(f>hi?hi:f); }
int GT_ValidateSafety(const GT_Safety *s) {
    unsigned i; uint8_t wire[GT_SAFETY_SIZE];
    GT_EncodeSafety(wire,s);
    for(i=0;i<26;i++) if(!finite_float(GT_ReadFloat(wire+4*i))) return 0;
    if(!(s->pos_min<s->pos_max) || s->margin<0 ||
       !(s->pos_min+s->margin<s->pos_max-s->margin)) return 0;
    /* Bounded magnitudes avoid overflowing arithmetic even before clamping. */
    for(i=0;i<26;i++) if(fabsf(GT_ReadFloat(wire+4*i))>1.0e6f) return 0;
    if(s->max_speed<=0 || s->max_output<=0 || s->max_output>32767 ||
       s->max_current<=0 || s->max_temperature<=0 || s->max_pair_error<=0 ||
       s->prepare_speed<=0 || s->prepare_speed>s->max_speed || s->prepare_accel<=0 ||
       s->settle_pos_tol<=0 || s->settle_speed_tol<=0 || s->other_pos_tol<=0 ||
       s->other_speed_tol<=0 || s->max_integral<=0 || s->max_integral>s->max_output) return 0;
    for(i=0;i<4;i++) if(s->gain_min[i]<0 || s->gain_max[i]<s->gain_min[i]) return 0;
    for(i=26;i<34;i++) {
        uint32_t v=GT_ReadU32(wire+4*i);
        if(!v || v>GT_MAX_DURATION_MS) return 0;
    }
    return s->feedback_timeout_ms<=60000 && s->heartbeat_timeout_ms<=60000 &&
        s->telemetry_period_ms>=5 && s->settle_hold_ms<=s->settle_timeout_ms &&
        s->max_trial_ms<=s->batch_timeout_ms && s->max_control_gap_us>=1000 && s->max_control_gap_us<=100000;
}
int GT_ValidateParams(const GT_Safety *s, const GT_Parameters *v) {
    float gains[4]; unsigned i;
    gains[0]=v->pos_p; gains[1]=v->pos_d; gains[2]=v->speed_p; gains[3]=v->speed_i;
    if(!v->version || v->algorithm>1) return 0;
    for(i=0;i<4;i++) if(!finite_float(gains[i]) || gains[i]<s->gain_min[i] || gains[i]>s->gain_max[i]) return 0;
    return 1;
}
float GT_PIDStep(GT_PIDState *s,float error,float p,float i,float d,float max_output,float max_integral,uint32_t algorithm) {
    float next_i=s->integral+i*error;
    float output_p=p*error, output_d=d*(error-s->last_error), raw;
    if(max_integral>0) next_i=clamp(next_i,-max_integral,max_integral);
    else if(algorithm==1) next_i=0; /* explicit zero-I allowance, legacy keeps original semantics */
    raw=(output_p+next_i)+output_d;
    /* Integrate if not driving further into saturation; permit unwinding. */
    if(algorithm==1 && ((raw>max_output && i*error>0) || (raw<-max_output && i*error<0))) next_i=s->integral;
    s->integral=next_i;
    s->last_error=error;
    /* Keep the legacy P+I+D operation order, not (P+D)+I. */
    s->output=clamp((output_p+next_i)+output_d,-max_output,max_output);
    return s->output;
}
void GT_ResetControl(GT_Context *c,float pos_ref,float vel_ref) {
    memset(&c->pos_pid,0,sizeof c->pos_pid); memset(&c->speed_pid,0,sizeof c->speed_pid);
    c->pos_pid.last_error=pos_ref-c->feedback.pos[0];
    c->speed_pid.last_error=vel_ref-c->feedback.vel[0];
}
static uint32_t elapsed_ms(const GT_Context *c,uint32_t then) { return (c->now_us-then)/1000u; }
static int moving_state(uint32_t state) { return state>=GT_PRECHECK && state<=GT_RUN; }
int GT_MotionAllowed(const GT_Context *c) {
    return c->boot_id && c->motion_enabled && !c->fault && c->batch_active && c->session;
}
int16_t GT_CANSlot(const GT_Context *c,uint8_t bus,uint16_t send_id,uint16_t id,int16_t original,uint8_t *force_frame) {
    if(bus!=0 || send_id!=0x1fe || (id!=0x205 && id!=0x206)) return original;
    *force_frame=1;
    return GT_MotionAllowed(c)?(int16_t)c->command[id-0x205]:0;
}
static void zero(GT_Context *c) { c->motion_enabled=0; c->output=0; c->command[0]=c->command[1]=0; }
static uint32_t status_flags(const GT_Context *c) {
    return (uint32_t)c->safety_valid | ((uint32_t)c->baseline_valid<<1) |
        ((uint32_t)c->staged_valid<<2) | ((uint32_t)c->batch_active<<3) | ((uint32_t)GT_MotionAllowed(c)<<4);
}
static void status_payload(const GT_Context *c,uint8_t *p) {
    const uint32_t v[12]={c->state,c->fault,c->active_version,c->applied_seq,c->sample_seq,
        c->trial_id,c->batch_id,status_flags(c),c->now_us,c->tx_dropped,c->rx_dropped,c->last_seq};
    unsigned i; for(i=0;i<12;i++) GT_WriteU32(p+4*i,v[i]);
}
static int emit(GT_Context *c,uint8_t type,uint32_t seq,const uint8_t *p,uint16_t len,int cache) {
    uint8_t out[GT_MAX_FRAME]; uint16_t n=GT_Encode(out,type,c->boot_id,c->session,seq,p,len);
    if(cache) { memcpy(c->last_reply,out,n); c->last_reply_len=n; }
    if(c->send && c->send(c->send_user,out,n)) return 1;
    c->tx_dropped++; return 0;
}
static void reply_impl(GT_Context *c,uint8_t command,uint32_t seq,uint32_t error,int cache) {
    uint8_t p[56]; GT_WriteU32(p,command); GT_WriteU32(p+4,error); status_payload(c,p+8);
    (void)emit(c,error?GT_NACK:GT_ACK,seq,p,sizeof p,cache);
}
static void reply(GT_Context *c,uint8_t command,uint32_t seq,uint32_t error) {
    reply_impl(c,command,seq,error,1);
}
static void change_state(GT_Context *c,uint32_t state) { c->state=state; c->state_at_us=c->now_us; }
static void result(GT_Context *c,uint32_t kind) {
    uint8_t p[76];
    GT_WriteU32(p,c->trial_id); GT_WriteU32(p+4,kind); GT_WriteU32(p+8,c->fault);
    GT_WriteU32(p+12,c->run_start_seq); GT_WriteU32(p+16,c->run_end_seq);
    GT_WriteU32(p+20,c->run_at_us); GT_WriteU32(p+24,c->run_end_us);
    status_payload(c,p+28); (void)emit(c,GT_RESULT,0,p,sizeof p,0);
}
static void fault(GT_Context *c,uint32_t reason) {
    int was_trial=moving_state(c->state); uint8_t p[48];
    if(c->fault) return;
    c->fault=reason; zero(c); c->batch_active=0; c->staged_valid=0; c->complete_pending=0;
    change_state(c,GT_FAULT_LATCHED_ZERO);
    if(was_trial) result(c,(reason==GT_F_PREPARE_TIMEOUT || reason==GT_F_SETTLE_TIMEOUT)?GT_R_PREPARATION_FAILED:GT_R_PROTECTION);
    status_payload(c,p); (void)emit(c,GT_FAULT,0,p,sizeof p,0);
}
void GT_TransportFault(GT_Context *c) { fault(c,GT_F_TRANSPORT); }
static int interior(const GT_Safety *s,float p) { return p>=s->pos_min+s->margin && p<=s->pos_max-s->margin; }
static uint32_t health(const GT_Context *c,int require_heartbeat) {
    unsigned i; const GT_Safety *s=&c->safety; const GT_Feedback *f=&c->feedback;
    if(require_heartbeat && (!c->heartbeat_valid || elapsed_ms(c,c->heartbeat_at_us)>s->heartbeat_timeout_ms)) return GT_F_HEARTBEAT;
    for(i=0;i<2;i++) {
        if(!f->online[i] || f->age_ms[i]>s->feedback_timeout_ms || !f->feedback_count[i] ||
           (f->direction[i]!=1 && f->direction[i]!=-1)) return GT_F_FEEDBACK;
        if(!finite_float(f->pos[i]) || !finite_float(f->vel[i]) || !finite_float(f->current[i]) || !finite_float(f->temperature[i])) return GT_F_NONFINITE;
        if(!interior(s,f->pos[i])) return GT_F_POSITION;
        if(fabsf(f->vel[i])>s->max_speed) return GT_F_SPEED;
        if(fabsf(f->current[i])>s->max_current) return GT_F_CURRENT;
        if(f->temperature[i]>s->max_temperature) return GT_F_TEMPERATURE;
    }
    if(fabsf(f->pos[0]-f->pos[1])>s->max_pair_error) return GT_F_PAIR_MISMATCH;
    if(f->external_stop) return GT_F_OTHER_JOINT;
    for(i=0;i<3;i++) {
        if(!f->other_online[i] || f->other_age_ms[i]>s->feedback_timeout_ms) return GT_F_OTHER_JOINT;
        if(!finite_float(f->other_pos[i]) || !finite_float(f->other_vel[i]) || !finite_float(f->other_command[i])) return GT_F_NONFINITE;
        if(fabsf(f->other_pos[i]-s->other_target[i])>s->other_pos_tol ||
           fabsf(f->other_command[i]-s->other_target[i])>s->other_pos_tol ||
           fabsf(f->other_vel[i])>s->other_speed_tol) return GT_F_OTHER_JOINT;
    }
    return GT_F_NONE;
}
static void apply(GT_Context *c,const GT_Parameters *v,int baseline) {
    c->active=*v; c->active_version=v->version; c->applied_seq=c->sample_seq; c->baseline_active=(uint8_t)baseline;
    GT_ResetControl(c,c->target_pos,0);
}
static void stop(GT_Context *c) {
    int was_trial=moving_state(c->state);
    zero(c); c->batch_active=0; c->staged_valid=0; c->complete_pending=0;
    if(!c->fault) change_state(c,GT_STOPPED_ZERO);
    if(was_trial) result(c,GT_R_STOPPED);
}
void GT_Init(GT_Context *c,uint32_t boot,GT_Send send,void *user) {
    memset(c,0,sizeof *c); c->boot_id=boot; c->send=send; c->send_user=user;
    if(!boot) { c->fault=GT_F_TRANSPORT; c->state=GT_FAULT_LATCHED_ZERO; }
}
static int zero_state(const GT_Context *c) { return !moving_state(c->state) && !c->motion_enabled; }
static void command(GT_Context *c,const uint8_t *frame,uint16_t total) {
    uint8_t type=frame[3], p[GT_MAX_PAYLOAD];
    const uint8_t *data=frame+18;
    uint16_t len=(uint16_t)(total-20);
    uint32_t boot=GT_ReadU32(frame+6),session=GT_ReadU32(frame+10),seq=GT_ReadU32(frame+14),err=0;
    GT_Parameters params; GT_Safety safety;
    if(type==GT_STOP && len==0) {
        stop(c); c->session=0; c->heartbeat_valid=0; c->last_seq=0;
        c->last_request_len=c->last_reply_len=0;
        reply(c,type,seq,0); return;
    }
    if(type==GT_HELLO && len==0 && boot==0 && session==0 && seq==0) {
        stop(c);
        if(c->session_counter==0xffffffffu) { c->session=0; fault(c,GT_F_TRANSPORT); return; }
        c->session=++c->session_counter; c->last_seq=0; c->heartbeat_valid=0;
        c->last_request_len=c->last_reply_len=0;
        c->highest_version=c->active_version>c->baseline.version?c->active_version:c->baseline.version;
        if(!c->fault) change_state(c,GT_IDLE_ZERO);
        GT_WriteU32(p,1); GT_WriteU32(p+4,65536); GT_WriteU32(p+8,3);
        GT_WriteU32(p+12,0x205); GT_WriteU32(p+16,0x206); GT_WriteU32(p+20,500000);
        GT_WriteU32(p+24,1000); GT_WriteU32(p+28,GT_TELEMETRY_SIZE);
        status_payload(c,p+32); (void)emit(c,GT_INFO,seq,p,80,0); return;
    }
    if(!c->session || session!=c->session || boot!=c->boot_id || !seq) { reply_impl(c,type,seq,GT_E_SESSION,0); return; }
    if(seq==c->last_seq && total==c->last_request_len && !memcmp(frame,c->last_request,total)) {
        if(c->last_reply_len && c->send && !c->send(c->send_user,c->last_reply,c->last_reply_len)) c->tx_dropped++;
        return; /* especially: no heartbeat refresh, no motion re-entry */
    }
    if(seq<=c->last_seq) { reply_impl(c,type,seq,GT_E_REPLAY,0); return; }
    /* Consume authenticated rejected attempts too: delayed START cannot become ready later. */
    c->last_seq=seq; memcpy(c->last_request,frame,total); c->last_request_len=total; c->last_reply_len=0;
    switch(type) {
    case GT_GET_STATUS:
        if(len) { err=GT_E_LENGTH; break; }
        status_payload(c,p); (void)emit(c,GT_STATUS,seq,p,48,1); return;
    case GT_GET_CONFIG:
        if(len) { err=GT_E_LENGTH; break; }
        memset(p,0,172); GT_WriteU32(p,c->safety_valid); GT_EncodeSafety(p+4,&c->safety);
        GT_WriteU32(p+144,c->baseline_valid); GT_EncodeParams(p+148,&c->baseline);
        (void)emit(c,GT_CONFIG,seq,p,172,1); return;
    case GT_HEARTBEAT:
        if(len) err=GT_E_LENGTH;
        else { c->heartbeat_valid=1; c->heartbeat_at_us=c->now_us; }
        break;
    case GT_SET_SAFETY:
        if(len!=GT_SAFETY_SIZE) { err=GT_E_LENGTH; break; }
        if(!zero_state(c) || c->batch_active) { err=GT_E_STATE; break; }
        GT_DecodeSafety(&safety,data);
        if(!GT_ValidateSafety(&safety)) { err=GT_E_CONFIG; break; }
        c->safety=safety; c->safety_valid=1;
        /* Changed limits invalidate previously accepted gains and staging. */
        c->baseline_valid=0; c->staged_valid=0;
        break;
    case GT_SET_BASELINE:
    case GT_STAGE:
        if(len!=GT_PARAM_SIZE) { err=GT_E_LENGTH; break; }
        if(!zero_state(c) || c->fault || (type==GT_SET_BASELINE && c->batch_active)) { err=GT_E_STATE; break; }
        GT_DecodeParams(&params,data);
        if(!c->safety_valid || !GT_ValidateParams(&c->safety,&params)) { err=GT_E_PARAMS; break; }
        if(type==GT_SET_BASELINE && c->baseline.version==params.version) {
            /* Identical baseline may be reinstalled after a limits transaction. */
            uint8_t old[24]; GT_EncodeParams(old,&c->baseline);
            if(memcmp(old,data,24)) { err=GT_E_PARAMS; break; }
        } else if(params.version<=c->highest_version) { err=GT_E_PARAMS; break; }
        if(type==GT_STAGE && c->baseline_valid && params.algorithm!=c->baseline.algorithm) { err=GT_E_PARAMS; break; }
        if(type==GT_SET_BASELINE) { c->baseline=params; c->baseline_valid=1; c->staged_valid=0; }
        else { c->staged=params; c->staged_valid=1; }
        if(params.version>c->highest_version) c->highest_version=params.version;
        break;
    case GT_START_BATCH:
        if(len!=8) { err=GT_E_LENGTH; break; }
        if(!zero_state(c) || c->batch_active || c->fault) { err=GT_E_STATE; break; }
        if(!c->boot_id || !c->safety_valid || !c->baseline_valid || !c->staged_valid || health(c,1)) { err=GT_E_READY; break; }
        if(!GT_ReadU32(data) || !GT_ReadU32(data+4)) { err=GT_E_BUDGET; break; }
        c->batch_id=GT_ReadU32(data); c->max_trials=GT_ReadU32(data+4);
        c->trials_started=0; c->last_trial_id=0; c->batch_active=1; c->batch_at_us=c->now_us;
        break;
    case GT_START_TRIAL:
        if(len!=20) { err=GT_E_LENGTH; break; }
        if(!zero_state(c) || c->fault || !c->batch_active || !c->staged_valid) { err=GT_E_STATE; break; }
        if(c->complete_pending) { err=GT_E_READY; break; }
        if(c->trials_started>=c->max_trials) { err=GT_E_BUDGET; break; }
        if(!GT_ReadU32(data) || GT_ReadU32(data)<=c->last_trial_id || GT_ReadU32(data+4)>1 ||
           !finite_float(GT_ReadFloat(data+8)) || !finite_float(GT_ReadFloat(data+12)) ||
           !interior(&c->safety,GT_ReadFloat(data+8)) || !GT_ReadU32(data+16) ||
           GT_ReadU32(data+16)>c->safety.max_trial_ms ||
           (GT_ReadU32(data+4)==0?!interior(&c->safety,GT_ReadFloat(data+12)):
                                  fabsf(GT_ReadFloat(data+12))>c->safety.max_speed)) { err=GT_E_TRIAL; break; }
        c->trial_id=GT_ReadU32(data); c->last_trial_id=c->trial_id; c->trials_started++;
        c->trial_mode=GT_ReadU32(data+4); c->origin=GT_ReadFloat(data+8); c->target=GT_ReadFloat(data+12);
        c->trial_duration_ms=GT_ReadU32(data+16); c->run_start_seq=c->run_end_seq=c->run_at_us=c->run_end_us=0;
        change_state(c,GT_PRECHECK); zero(c);
        break;
    case GT_CLEAR_FAULT:
        if(len) { err=GT_E_LENGTH; break; }
        if(!zero_state(c) || !c->fault) { err=GT_E_STATE; break; }
        if(!c->boot_id || !c->safety_valid || health(c,1) || c->dt_us>c->safety.max_control_gap_us) { err=GT_E_READY; break; }
        c->fault=0; c->batch_active=0; c->staged_valid=0; zero(c); change_state(c,GT_IDLE_ZERO);
        break;
    case GT_END_BATCH:
        if(len) { err=GT_E_LENGTH; break; }
        if(!zero_state(c)) { err=GT_E_STATE; break; }
        c->batch_active=0; c->batch_id=0; c->staged_valid=0; c->complete_pending=0; zero(c);
        if(!c->fault) change_state(c,GT_IDLE_ZERO);
        break;
    default: err=GT_E_COMMAND; break;
    }
    reply(c,type,seq,err);
}
static void discard_byte(GT_Context *c) {
    if(c->rx_len) { c->rx_len--; memmove(c->rx,c->rx+1,c->rx_len); }
}
void GT_Feed(GT_Context *c,uint8_t byte,uint32_t now) {
    uint16_t len,total,crc;
    c->now_us=now;
    if(c->rx_len && now-c->rx_at_us>GT_FRAME_TIMEOUT_US) { c->rx_len=0; c->rx_dropped++; }
    if(!c->rx_len) c->rx_at_us=now;
    if(c->rx_len>=GT_MAX_FRAME) { discard_byte(c); c->rx_dropped++; }
    c->rx[c->rx_len++]=byte;
    while(c->rx_len>=2) {
        if(c->rx[0]!=0xa5 || c->rx[1]!=0x5a) { discard_byte(c); continue; }
        if(c->rx_len<6) return;
        len=(uint16_t)(c->rx[4]|((uint16_t)c->rx[5]<<8));
        if(c->rx[2]!=GT_PROTOCOL || len>GT_MAX_PAYLOAD) { discard_byte(c); c->rx_dropped++; continue; }
        total=(uint16_t)(len+20); if(c->rx_len<total) return;
        crc=GT_CRC16(c->rx+2,(size_t)len+16);
        if(c->rx[total-2]!=(uint8_t)crc || c->rx[total-1]!=(uint8_t)(crc>>8)) { discard_byte(c); c->rx_dropped++; continue; }
        command(c,c->rx,total); c->rx_len=(uint16_t)(c->rx_len-total);
        if(c->rx_len) memmove(c->rx,c->rx+total,c->rx_len);
        c->rx_at_us=now;
    }
}
static void telemetry(GT_Context *c) {
    uint8_t p[124]; unsigned i;
    uint32_t flags=(uint32_t)c->saturated | ((uint32_t)GT_MotionAllowed(c)<<1) |
        ((uint32_t)c->baseline_active<<2) | ((uint32_t)(c->state==GT_RUN)<<3);
    const uint32_t u[15]={c->now_us,c->sample_seq,c->trial_id,c->state,c->fault,c->active_version,c->applied_seq,
        c->tx_dropped,c->rx_dropped,c->dt_us,c->feedback.age_ms[0],c->feedback.age_ms[1],
        c->feedback.feedback_count[0],c->feedback.feedback_count[1],flags};
    const float f[16]={c->target_pos,c->target_vel,c->feedback.pos[0],c->feedback.pos[1],c->feedback.vel[0],c->feedback.vel[1],
        c->feedback.current[0],c->feedback.current[1],c->feedback.temperature[0],c->feedback.temperature[1],
        c->pos_error,c->speed_error,c->speed_pid.integral,c->output,c->command[0],c->command[1]};
    for(i=0;i<15;i++) GT_WriteU32(p+4*i,u[i]);
    for(i=0;i<16;i++) GT_WriteFloat(p+60+4*i,f[i]);
    (void)emit(c,GT_TELEMETRY,0,p,sizeof p,0); c->last_telemetry_us=c->now_us;
}
static void prepare_reference(GT_Context *c) {
    float t=(c->now_us-c->state_at_us)*1.0e-6f, a=c->safety.prepare_accel;
    float ta=c->prepare_accel_time, tc=c->prepare_coast_time, end=2*ta+tc, x,v;
    if(t>=end) { x=c->prepare_distance; v=0; }
    else if(t<ta) { x=0.5f*a*t*t; v=a*t; }
    else if(t<ta+tc) { x=0.5f*a*ta*ta+c->prepare_peak_speed*(t-ta); v=c->prepare_peak_speed; }
    else { float remaining=end-t; x=c->prepare_distance-0.5f*a*remaining*remaining; v=a*remaining; }
    c->target_pos=c->prepare_origin+c->prepare_sign*x;
    /* This is path velocity; the actual inner-loop reference is set by position PID. */
    c->target_vel=c->prepare_sign*v;
    if(t>=end) { c->target_pos=c->origin; change_state(c,GT_SETTLE); c->settle_tracking=0; }
}
void GT_Tick(GT_Context *c,uint32_t now,const GT_Feedback *feedback) {
    uint32_t reason; int force_telem=0,finish=0; unsigned i;
    c->dt_us=c->have_tick?now-c->last_tick_us:1000u; c->have_tick=1;
    c->last_tick_us=now; c->now_us=now; c->sample_seq++; c->feedback=*feedback;
    if(c->complete_pending && c->state==GT_COMPLETE_ZERO) { force_telem=1; c->complete_pending=0; }
    if(c->rx_len && now-c->rx_at_us>GT_FRAME_TIMEOUT_US) { c->rx_len=0; c->rx_dropped++; }
    if(c->batch_active && !c->fault) {
        reason=health(c,1);
        if(!reason && c->dt_us>c->safety.max_control_gap_us) reason=GT_F_CONTROL_TIMING;
        if(!reason && elapsed_ms(c,c->batch_at_us)>=c->safety.batch_timeout_ms) reason=GT_F_BATCH_TIMEOUT;
        if(reason) fault(c,reason);
    }
    if(c->state==GT_PRECHECK && !c->fault) {
        /* Position starts from measured state, never from previous trial's assumed endpoint. */
        float distance=c->origin-c->feedback.pos[0], ta;
        c->prepare_origin=c->feedback.pos[0]; c->prepare_distance=fabsf(distance); c->prepare_sign=distance<0?-1.0f:1.0f;
        ta=sqrtf(c->prepare_distance/c->safety.prepare_accel);
        if(ta>c->safety.prepare_speed/c->safety.prepare_accel) ta=c->safety.prepare_speed/c->safety.prepare_accel;
        c->prepare_accel_time=ta; c->prepare_peak_speed=c->safety.prepare_accel*ta;
        c->prepare_coast_time=c->prepare_peak_speed>0?fmaxf(0,c->prepare_distance/c->prepare_peak_speed-ta):0;
        if(!finite_float(ta) || !finite_float(c->prepare_peak_speed) || !finite_float(c->prepare_coast_time) ||
           (c->prepare_distance>0 && c->prepare_peak_speed<=0)) fault(c,GT_F_NONFINITE);
        else if((2*ta+c->prepare_coast_time)*1000.0f>(float)c->safety.prepare_timeout_ms) fault(c,GT_F_PREPARE_TIMEOUT);
        else {
            c->target_pos=c->prepare_origin; c->target_vel=0; apply(c,&c->baseline,1);
            c->motion_enabled=1; change_state(c,GT_PREPARE_BASELINE);
        }
    }
    if(c->state==GT_PREPARE_BASELINE) {
        if(elapsed_ms(c,c->state_at_us)>c->safety.prepare_timeout_ms) fault(c,GT_F_PREPARE_TIMEOUT);
        else prepare_reference(c);
    }
    if(c->state==GT_SETTLE) {
        int stable=1;
        c->target_pos=c->origin;
        for(i=0;i<2;i++) if(fabsf(c->feedback.pos[i]-c->origin)>c->safety.settle_pos_tol ||
                            fabsf(c->feedback.vel[i])>c->safety.settle_speed_tol) stable=0;
        if(elapsed_ms(c,c->state_at_us)>c->safety.settle_timeout_ms) fault(c,GT_F_SETTLE_TIMEOUT);
        else if(!stable) c->settle_tracking=0;
        else if(!c->settle_tracking) { c->settle_tracking=1; c->settle_at_us=now; }
        else if(elapsed_ms(c,c->settle_at_us)>=c->safety.settle_hold_ms) change_state(c,GT_APPLY_CANDIDATE);
    }
    /* Apply in the SAME sampled boundary that proved the settle hold. A separate
     * tick would let motion restart between the rest check and history reset. */
    if(c->state==GT_APPLY_CANDIDATE) {
        c->target_pos=c->origin; c->target_vel=0; apply(c,&c->staged,0); c->staged_valid=0;
        c->run_at_us=now; c->run_start_seq=c->sample_seq; change_state(c,GT_RUN); force_telem=1;
    }
    if(c->state==GT_RUN) {
        if(c->trial_mode==0) c->target_pos=c->target;
        else {
            float limit=c->safety.prepare_accel*(now-c->run_at_us)*1.0e-6f;
            c->target_vel=clamp(c->target,-limit,limit); c->target_pos=c->origin;
        }
        if(elapsed_ms(c,c->run_at_us)>=c->trial_duration_ms) {
            c->run_end_seq=c->sample_seq; c->run_end_us=now; force_telem=1; finish=1;
        }
    }
    if(GT_MotionAllowed(c)) {
        float vel_ref,raw;
        c->pos_error=c->target_pos-c->feedback.pos[0];
        if(c->state==GT_RUN && c->trial_mode==1) vel_ref=c->target_vel;
        else vel_ref=GT_PIDStep(&c->pos_pid,c->pos_error,c->active.pos_p,0,c->active.pos_d,c->safety.max_speed,1,0);
        c->target_vel=vel_ref; c->speed_error=vel_ref-c->feedback.vel[0];
        raw=c->active.speed_p*c->speed_error+c->speed_pid.integral+c->active.speed_i*c->speed_error;
        c->output=GT_PIDStep(&c->speed_pid,c->speed_error,c->active.speed_p,c->active.speed_i,0,
                            c->safety.max_output,c->safety.max_integral,c->active.algorithm);
        c->saturated=(uint8_t)(fabsf(raw)>=c->safety.max_output || fabsf(vel_ref)>=c->safety.max_speed ||
            fabsf(c->speed_pid.integral)>=c->safety.max_integral);
        if(!finite_float(c->output) || !finite_float(c->speed_pid.integral)) fault(c,GT_F_NONFINITE);
        else for(i=0;i<2;i++) c->command[i]=(float)(int16_t)(c->output*0.5f*c->feedback.direction[i]);
    } else { zero(c); c->saturated=0; }
    /* Final RUN endpoint records the command actually handed to the CAN adapter. */
    if(finish) zero(c);
    if(force_telem || (c->safety_valid && elapsed_ms(c,c->last_telemetry_us)>=c->safety.telemetry_period_ms)) telemetry(c);
    if(finish && !c->fault) {
        change_state(c,GT_COMPLETE_ZERO); c->complete_pending=1; result(c,GT_R_COMPLETE);
        /* Remain zero. No auto-loading or implicit re-prepare. */
    }
}
