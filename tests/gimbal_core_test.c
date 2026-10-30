#include "Driver_GimbalTuning.h"
#include "Driver_PID.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static GT_Context c;
static GT_Feedback fb;
static uint32_t seq, now;
static uint8_t last[GT_MAX_FRAME], endpoint[GT_TELEMETRY_SIZE], saved_result[76];
static uint16_t last_len;
static unsigned results, telemetry_count;
static int send_frame(void *user,const uint8_t *frame,uint16_t n) {
    (void)user; memcpy(last,frame,n); last_len=n;
    if(frame[3]==GT_RESULT) { memcpy(saved_result,frame+18,76); results++; }
    if(frame[3]==GT_TELEMETRY) { memcpy(endpoint,frame+18,124); telemetry_count++; }
    return 1;
}
static void feed(const uint8_t *p,uint16_t n) { unsigned i; for(i=0;i<n;i++) GT_Feed(&c,p[i],now); }
static uint16_t make(uint8_t *frame,uint8_t type,const uint8_t *p,uint16_t n) {
    return GT_Encode(frame,type,c.boot_id,c.session,++seq,p,n);
}
static void cmd(uint8_t type,const uint8_t *p,uint16_t n) { uint8_t wire[276]; uint16_t len=make(wire,type,p,n); feed(wire,len); }
static void tick(uint32_t dt) {
    now+=dt; fb.feedback_count[0]++; fb.feedback_count[1]++; GT_Tick(&c,now,&fb);
}
static GT_Safety safety(void) {
    GT_Safety s; unsigned i; memset(&s,0,sizeof s);
    s.pos_min=-1; s.pos_max=1; s.margin=.05f; s.max_speed=5; s.max_output=4000;
    s.max_current=3; s.max_temperature=80; s.max_pair_error=.1f;
    s.prepare_speed=1; s.prepare_accel=2; s.settle_pos_tol=.005f; s.settle_speed_tol=.05f;
    s.other_pos_tol=.01f; s.other_speed_tol=.1f; s.max_integral=1000;
    for(i=0;i<4;i++) s.gain_max[i]=1000;
    s.feedback_timeout_ms=20; s.heartbeat_timeout_ms=200; s.prepare_timeout_ms=2000;
    s.settle_timeout_ms=1000; s.settle_hold_ms=10; s.max_trial_ms=1000;
    s.telemetry_period_ms=5; s.batch_timeout_ms=10000; s.max_control_gap_us=5000;
    return s;
}
static void init(void) {
    unsigned i; uint8_t frame[276]; uint16_t n;
    GT_Init(&c,0x12345678u,send_frame,NULL); seq=now=0; results=telemetry_count=0;
    memset(&fb,0,sizeof fb); memset(endpoint,0,sizeof endpoint); memset(saved_result,0,sizeof saved_result);
    for(i=0;i<2;i++) { fb.online[i]=1; fb.direction[i]=1; fb.feedback_count[i]=1; fb.temperature[i]=25; }
    for(i=0;i<3;i++) fb.other_online[i]=1;
    tick(1000); assert(!GT_MotionAllowed(&c) && c.command[0]==0 && c.command[1]==0);
    n=GT_Encode(frame,GT_HELLO,0,0,0,NULL,0); feed(frame,n);
    assert(c.session==1 && last[3]==GT_INFO && last_len==100);
}
static void configure(void) {
    uint8_t p[140]; GT_Safety s=safety(); GT_Parameters v={1,1,0,50,.5f,0};
    GT_EncodeSafety(p,&s); cmd(GT_SET_SAFETY,p,140); assert(last[3]==GT_ACK);
    GT_EncodeParams(p,&v); cmd(GT_SET_BASELINE,p,24); assert(last[3]==GT_ACK);
    v.version=2; GT_EncodeParams(p,&v); cmd(GT_STAGE,p,24); assert(last[3]==GT_ACK);
    assert(c.active_version==0); cmd(GT_HEARTBEAT,NULL,0);
}
static void batch(void) {
    uint8_t p[8]; GT_WriteU32(p,7); GT_WriteU32(p+4,4); cmd(GT_START_BATCH,p,8);
    assert(last[3]==GT_ACK && c.batch_active && !GT_MotionAllowed(&c));
}
static void trial(float origin,float target) {
    uint8_t p[20]; GT_WriteU32(p,1); GT_WriteU32(p+4,0); GT_WriteFloat(p+8,origin);
    GT_WriteFloat(p+12,target); GT_WriteU32(p+16,50); cmd(GT_START_TRIAL,p,20);
    assert(last[3]==GT_ACK && c.state==GT_PRECHECK);
}
static void run_to_window(void) {
    unsigned i;
    for(i=0;i<100 && c.state!=GT_RUN;i++) tick(1000);
    assert(c.state==GT_RUN && c.active_version==2 && c.applied_seq==c.run_start_seq && !c.staged_valid);
}
static void test_codec(void) {
    uint8_t p[140], q[140], frame[276]; GT_Safety s=safety(), copy;
    assert(GT_CRC16((const uint8_t *)"123456789",9)==0x29b1);
    assert(GT_ValidateSafety(&s)); GT_EncodeSafety(p,&s); GT_DecodeSafety(&copy,p);
    GT_EncodeSafety(q,&copy); assert(!memcmp(p,q,140));
    assert(GT_Encode(frame,GT_SET_SAFETY,1,2,3,p,140)==160);
    s.max_current=NAN; assert(!GT_ValidateSafety(&s)); s=safety();
    s.batch_timeout_ms=1800001; assert(!GT_ValidateSafety(&s)); s=safety();
    s.gain_max[0]=INFINITY; assert(!GT_ValidateSafety(&s));
    s=safety(); s.margin=1; assert(!GT_ValidateSafety(&s));
    s=safety(); s.max_integral=0; assert(!GT_ValidateSafety(&s));
}
static void test_parser(void) {
    uint8_t frame[276], bad[276]; uint16_t n; unsigned i;
    init(); n=make(frame,GT_GET_STATUS,NULL,0); memcpy(bad,frame,n); bad[18]^=1;
    feed(bad,n); feed(frame,n); assert(last[3]==GT_STATUS && c.rx_dropped>0);
    n=make(frame,GT_GET_STATUS,NULL,0); memcpy(bad,frame,n); bad[2]=9;
    feed(bad,n); feed(frame,n); assert(last[3]==GT_STATUS);
    for(i=0;i<6;i++) GT_Feed(&c,frame[i],now);
    now+=GT_FRAME_TIMEOUT_US+1; n=make(frame,GT_GET_STATUS,NULL,0); feed(frame,n);
    assert(last[3]==GT_STATUS && c.rx_len==0);
    bad[0]=0xa5; bad[1]=0x5a; bad[2]=1; bad[4]=255; bad[5]=255;
    feed(bad,6); n=make(frame,GT_GET_STATUS,NULL,0); feed(frame,n); assert(last[3]==GT_STATUS);
}
static void test_complete_and_transaction(void) {
    uint32_t start; init(); configure(); batch(); trial(0,.01f);
    tick(1000); assert(c.active_version==1 && c.baseline_active);
    run_to_window(); start=c.run_start_seq;
    while(c.state==GT_RUN) tick(1000);
    assert(c.state==GT_COMPLETE_ZERO && !GT_MotionAllowed(&c) && c.command[0]==0 && c.command[1]==0);
    assert(results==1 && GT_ReadU32(saved_result+4)==GT_R_COMPLETE && GT_ReadU32(saved_result+12)==start);
    assert(GT_ReadU32(saved_result+16)==c.run_end_seq && GT_ReadU32(endpoint+4)==c.run_end_seq);
    assert(GT_ReadU32(endpoint+12)==GT_RUN && GT_ReadFloat(endpoint+116)==0 && GT_ReadFloat(endpoint+120)==0);
    assert(c.batch_active); tick(1000); assert(c.state==GT_COMPLETE_ZERO && c.output==0);
    cmd(GT_END_BATCH,NULL,0); assert(!c.batch_active && !c.batch_id && c.state==GT_IDLE_ZERO);
}
static void test_replay_and_stop(void) {
    uint8_t frame[276], old[276], p[20]; uint16_t n,on; uint32_t hb;
    init(); configure(); batch();
    GT_WriteU32(p,1); GT_WriteU32(p+4,0); GT_WriteFloat(p+8,0); GT_WriteFloat(p+12,.01f); GT_WriteU32(p+16,50);
    n=make(frame,GT_START_TRIAL,p,20); feed(frame,n); tick(1000); feed(frame,n);
    assert(c.trials_started==1 && last[3]==GT_ACK);
    /* Rejected old command must not poison the last accepted response cache. */
    on=GT_Encode(old,GT_GET_STATUS,c.boot_id,c.session,seq-1,NULL,0); feed(old,on);
    assert(last[3]==GT_NACK); feed(frame,n); assert(last[3]==GT_ACK && GT_ReadU32(last+18)==GT_START_TRIAL);
    n=make(frame,GT_HEARTBEAT,NULL,0); feed(frame,n); hb=c.heartbeat_at_us;
    tick(1000); feed(frame,n); assert(c.heartbeat_at_us==hb);
    cmd(GT_STOP,NULL,0); assert(!c.session && !c.batch_active && !GT_MotionAllowed(&c) && c.command[1]==0);
    on=GT_Encode(old,GT_START_TRIAL,c.boot_id,1,seq+100,p,20); feed(old,on);
    assert(last[3]==GT_NACK && c.state==GT_STOPPED_ZERO);
    n=GT_Encode(frame,GT_HELLO,0,0,0,NULL,0); feed(frame,n);
    assert(c.session==2 && !c.batch_active && c.state==GT_IDLE_ZERO); feed(old,on); assert(last[3]==GT_NACK);
}
static void test_protection(void) {
    unsigned kind;
    for(kind=0;kind<9;kind++) {
        init(); configure(); batch(); trial(0,.01f); tick(1000);
        switch(kind) {
        case 0: fb.age_ms[1]=21; break;
        case 1: fb.online[1]=0; break;
        case 2: fb.pos[1]=.96f; break;
        case 3: fb.vel[0]=6; break;
        case 4: fb.current[1]=4; break;
        case 5: fb.temperature[0]=90; break;
        case 6: fb.pos[1]=.2f; break;
        case 7: fb.other_command[1]=.1f; break;
        case 8: fb.pos[0]=NAN; break;
        }
        tick(1000); assert(c.state==GT_FAULT_LATCHED_ZERO && c.fault && !c.batch_active);
        assert(c.command[0]==0 && c.command[1]==0 && results==1);
        cmd(GT_CLEAR_FAULT,NULL,0); assert(last[3]==GT_NACK && c.fault);
    }
    init(); configure(); batch(); trial(0,.01f); tick(1000); tick(6000);
    assert(c.fault==GT_F_CONTROL_TIMING && c.command[1]==0);
    tick(1000); cmd(GT_HEARTBEAT,NULL,0); cmd(GT_CLEAR_FAULT,NULL,0);
    assert(last[3]==GT_ACK && !c.fault && !GT_MotionAllowed(&c) && !c.batch_active);
    init(); configure(); batch();
    for(kind=0;kind<210;kind++) tick(1000);
    assert(c.fault==GT_F_HEARTBEAT);
}
static void test_preparation_failure(void) {
    unsigned i; init(); configure(); c.safety.prepare_timeout_ms=5; batch(); trial(.5f,.51f);
    for(i=0;i<8;i++) tick(1000);
    assert(c.fault==GT_F_PREPARE_TIMEOUT && GT_ReadU32(saved_result+4)==GT_R_PREPARATION_FAILED);
    init(); configure(); c.safety.settle_timeout_ms=10; batch(); trial(.001f,.01f);
    /* Configured tolerance allows origin; now inject a small nonsettling speed. */
    fb.vel[0]=.1f;
    for(i=0;i<200 && !c.fault;i++) tick(1000);
    assert(c.fault==GT_F_SETTLE_TIMEOUT && c.command[0]==0);
    init(); configure(); c.safety.prepare_accel=1.0e-40f; batch(); trial(.2f,.21f); tick(1000);
    assert(c.fault==GT_F_NONFINITE && !GT_MotionAllowed(&c) && c.command[0]==0);
}
static void test_reference_limits_and_zero_barrier(void) {
    unsigned n; float previous=0,previous_v=0;
    init(); configure(); batch(); trial(.2f,.21f); tick(1000);
    /* Reference-generator test with synthetic following feedback, NOT a plant. */
    for(n=0;n<1500 && c.state==GT_PREPARE_BASELINE;n++) {
        float v,a;
        if(n%50==0) cmd(GT_HEARTBEAT,NULL,0);
        fb.pos[0]=fb.pos[1]=previous; fb.vel[0]=fb.vel[1]=previous_v;
        tick(1000);
        v=(c.target_pos-previous)*1000; a=(v-previous_v)*1000;
        assert(c.target_pos>=previous-.000001f && c.target_pos<=.200001f);
        assert(fabsf(v)<=c.safety.prepare_speed+.005f && fabsf(a)<=c.safety.prepare_accel+.05f);
        previous=c.target_pos; previous_v=v;
    }
    assert(c.state==GT_SETTLE && fabsf(c.target_pos-.2f)<.000001f);
    init(); configure(); batch(); trial(0,.01f); run_to_window();
    while(c.state==GT_RUN) tick(1000);
    {
        uint8_t p[24]; GT_Parameters v=c.active; v.version=3;
        GT_EncodeParams(p,&v); cmd(GT_STAGE,p,24); assert(last[3]==GT_ACK);
        GT_WriteU32(p,2); GT_WriteU32(p+4,0); GT_WriteFloat(p+8,0); GT_WriteFloat(p+12,.01f); GT_WriteU32(p+16,50);
        cmd(GT_START_TRIAL,p,20); assert(last[3]==GT_NACK && GT_ReadU32(last+22)==GT_E_READY);
        tick(1000); assert(!c.complete_pending && GT_ReadU32(endpoint+12)==GT_COMPLETE_ZERO);
        cmd(GT_START_TRIAL,p,20); assert(last[3]==GT_ACK && c.state==GT_PRECHECK);
    }
}
static void test_apply_uses_the_settled_sample(void) {
    unsigned i;
    init(); configure(); batch(); trial(0,.01f); tick(1000);
    assert(c.state==GT_SETTLE && c.active_version==1);
    for(i=0;i<9;i++) tick(1000);
    fb.vel[0]=.1f; tick(1000);
    assert(c.state==GT_SETTLE && c.active_version==1 && !c.settle_tracking);
    fb.vel[0]=0; tick(1000);
    for(i=0;i<9;i++) tick(1000);
    assert(c.state==GT_SETTLE && c.active_version==1);
    tick(1000);
    assert(c.state==GT_RUN && c.active_version==2 && c.applied_seq==c.sample_seq);
}
static void test_microsecond_wrap(void) {
    init(); now=0xffff8000u; tick(1000); configure(); batch(); trial(0,.01f); run_to_window();
    while(c.state==GT_RUN) tick(1000);
    assert(!c.fault && c.run_end_us<c.run_at_us && (uint32_t)(c.run_end_us-c.run_at_us)==50000u);
    assert(c.state==GT_COMPLETE_ZERO && c.command[0]==0 && c.command[1]==0);
}
static void test_rejected_parameter_transactions(void) {
    uint8_t p[140]; GT_Parameters bad; GT_Safety invalid;
    init(); configure(); bad=c.staged; bad.version=3; bad.speed_i=NAN;
    GT_EncodeParams(p,&bad); cmd(GT_STAGE,p,24);
    assert(last[3]==GT_NACK && c.staged.version==2 && c.active_version==0);
    bad=c.staged; bad.version=3; bad.pos_p=c.safety.gain_max[0]+1;
    GT_EncodeParams(p,&bad); cmd(GT_STAGE,p,24);
    assert(last[3]==GT_NACK && c.staged.version==2);
    invalid=c.safety; invalid.max_output=0;
    GT_EncodeSafety(p,&invalid); cmd(GT_SET_SAFETY,p,140);
    assert(last[3]==GT_NACK && c.safety.max_output==4000 && c.baseline_valid);
    batch(); trial(0,.01f); tick(1000);
    bad=c.staged; bad.version=3; GT_EncodeParams(p,&bad); cmd(GT_STAGE,p,24);
    assert(last[3]==GT_NACK && c.active_version==1); /* no mixed live gains */
    GT_EncodeSafety(p,&c.safety); cmd(GT_SET_SAFETY,p,140);
    assert(last[3]==GT_NACK); /* cannot loosen limits during an authorized batch */
    GT_TransportFault(&c); cmd(GT_STOP,NULL,0);
    assert(c.fault==GT_F_TRANSPORT && c.state==GT_FAULT_LATCHED_ZERO);
}
static void test_final_can_slots(void) {
    uint8_t forced=0;
    init();
    assert(GT_CANSlot(&c,0,0x1fe,0x205,100,&forced)==0 && forced);
    assert(GT_CANSlot(&c,0,0x1fe,0x206,100,&forced)==0);
    forced=0;
    assert(GT_CANSlot(&c,0,0x1fe,0x208,1234,&forced)==1234 && !forced);
    assert(GT_CANSlot(&c,1,0x1fe,0x205,1234,&forced)==1234 && !forced);
    assert(GT_CANSlot(&c,0,0x1ff,0x205,1234,&forced)==1234 && !forced);
    configure(); batch(); trial(0,.1f); fb.direction[1]=-1; run_to_window();
    assert(c.command[0]==-c.command[1]);
    assert(GT_CANSlot(&c,0,0x1fe,0x205,0,&forced)==(int16_t)c.command[0]);
    cmd(GT_STOP,NULL,0); assert(GT_CANSlot(&c,0,0x1fe,0x206,100,&forced)==0);
}
static void test_pid(void) {
    GT_PIDState a={0,0,0},b={0,0,0}; unsigned i;
    PID_Type legacy;
    PID_Init(&legacy,800,16,0,16384,10000);
    for(i=0;i<1000;i++) {
        float error=sinf((float)i*.05f)*3;
        float old=PID_Calculate(&legacy,error,0);
        float current=GT_PIDStep(&a,error,800,16,0,16384,10000,0);
        assert(old==current && legacy.output_I==a.integral);
    }
    PID_Init(&legacy,1.8f,0,25,2,1); memset(&a,0,sizeof a);
    for(i=0;i<1000;i++) {
        float error=cosf((float)i*.05f)*.3f;
        assert(PID_Calculate(&legacy,error,0)==GT_PIDStep(&a,error,1.8f,0,25,2,1,0));
    }
    memset(&a,0,sizeof a);
    assert(GT_PIDStep(&a,2,3,.5f,4,100,10,0)==15);
    assert(GT_PIDStep(&a,1,3,.5f,4,100,10,0)==.5f);
    memset(&a,0,sizeof a);
    for(i=0;i<10;i++) { GT_PIDStep(&a,10,10,1,0,20,15,0); GT_PIDStep(&b,10,10,1,0,20,15,1); }
    assert(a.integral==15 && b.integral==0);
    b.integral=10; GT_PIDStep(&b,-1,0,1,0,20,15,1); assert(b.integral==9);
}
int main(void) {
    test_codec(); test_parser(); test_complete_and_transaction(); test_replay_and_stop();
    test_protection(); test_preparation_failure(); test_reference_limits_and_zero_barrier();
    test_rejected_parameter_transactions(); test_apply_uses_the_settled_sample(); test_microsecond_wrap();
    test_final_can_slots(); test_pid();
    puts("gimbal core: codec, transactions, replay, STOP, protection, trajectories and PID passed");
    return 0;
}
