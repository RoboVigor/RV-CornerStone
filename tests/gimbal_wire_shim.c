/* Host-only ABI around the actual portable firmware core for Python/C tests.
 * No platform code, hardware access, dependencies or protocol reimplementation. */
#include "Driver_GimbalTuning.h"
#include <string.h>
static GT_Context device;
static GT_Feedback feedback;
static uint8_t bytes[65536];
static uint32_t used;
static int capture(void *user,const uint8_t *p,uint16_t n) {
    (void)user; if(used+n>sizeof bytes) return 0;
    memcpy(bytes+used,p,n); used+=n; return 1;
}
void gts_reset(uint32_t boot) {
    unsigned i; used=0; GT_Init(&device,boot,capture,0);
    memset(&feedback,0,sizeof feedback);
    for(i=0;i<2;i++) { feedback.online[i]=1; feedback.direction[i]=1; feedback.feedback_count[i]=1; feedback.temperature[i]=25; }
    for(i=0;i<3;i++) feedback.other_online[i]=1;
    GT_Tick(&device,1000,&feedback);
}
void gts_feed(const uint8_t *p,uint32_t n,uint32_t now) {
    uint32_t i; for(i=0;i<n;i++) GT_Feed(&device,p[i],now);
}
void gts_tick(uint32_t now) {
    feedback.feedback_count[0]++; feedback.feedback_count[1]++;
    GT_Tick(&device,now,&feedback);
}
void gts_offline(uint32_t motor) { if(motor<2) feedback.online[motor]=0; }
uint32_t gts_read(uint8_t *p,uint32_t capacity) {
    uint32_t n=used<capacity?used:capacity; memcpy(p,bytes,n); used-=n;
    if(used) memmove(bytes,bytes+n,used);
    return n;
}
