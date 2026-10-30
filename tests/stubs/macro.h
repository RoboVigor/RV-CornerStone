/* Host-only shim for comparing the actual legacy Driver_PID.c arithmetic. */
#ifndef GIMBAL_TEST_MACRO_H
#define GIMBAL_TEST_MACRO_H
#include <stdio.h>
#define MIAO(v, lo, hi) do { if ((v)<(lo)) (v)=(lo); if ((v)>(hi)) (v)=(hi); } while (0)
#endif
