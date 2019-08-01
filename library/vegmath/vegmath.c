/**
 * @file  vegmath.c
 * @brief 绿色数学库
 * @note  我数学很菜(;´Д`)
 *        学数学是不可能学的,
 *        就是抄一些代码,
 *        才能维持的了生活这样子.
 */

#include <math.h>

#define PI 3.1415926f

#define cordic_1K 0x26DD3B6A
int cordic_ctab[] = {
    0x3243F6A8, 0x1DAC6705, 0x0FADBAFC, 0x07F56EA6, 0x03FEAB76, 0x01FFD55B, 0x00FFFAAA, 0x007FFF55, 0x003FFFEA, 0x001FFFFD, 0x000FFFFF,
    0x0007FFFF, 0x0003FFFF, 0x0001FFFF, 0x0000FFFF, 0x00007FFF, 0x00003FFF, 0x00001FFF, 0x00000FFF, 0x000007FF, 0x000003FF, 0x000001FF,
    0x000000FF, 0x0000007F, 0x0000003F, 0x0000001F, 0x0000000F, 0x00000008, 0x00000004, 0x00000002, 0x00000001, 0x00000000,
};

void cordic(int theta, int *s, int *c) {
    int k, d, tx, ty, tz;
    int x = cordic_1K, y = 0, z = theta;
    for (k = 0; k < 32; ++k) {
        d  = z >= 0 ? 0 : -1;
        tx = x - (((y >> k) ^ d) - d);
        ty = y + (((x >> k) ^ d) - d);
        tz = z - ((cordic_ctab[k] ^ d) - d);
        x  = tx;
        y  = ty;
        z  = tz;
    }
    *c = x;
    *s = y;
}

double vegsin(float deg) {
    int s, c;
    deg = fmod(fabs(deg + 90), 360);
    deg = deg > 180 ? 270 - deg : deg - 90;
    cordic(deg * 18740330.14516947, &s, &c); // 18740330.14516947 = 1/90* M_PI/2*1073741824;
    return s / 1073741824.0;
}

double vegcos(float deg) {
    return vegsin(deg + 90);
}

float FirstOrderLowPassFilter(float input, float *output, float sampleFrq, float cutFrq) {
    float RC, Cof1, Cof2;
    RC      = (float) 1.0 / 2.0 / PI / cutFrq;
    Cof1    = 1 / (1 + RC * sampleFrq);
    Cof2    = RC * sampleFrq / (1 + RC * sampleFrq);
    *output = Cof1 * input + Cof2 * (*output);
    return *output;
}

int FastLog2(int x) {
    float         fx;
    unsigned long ix, exp;

    fx  = (float) x;
    ix  = *(unsigned long *) &fx;
    exp = (ix >> 23) & 0xFF;

    return exp - 127;
}
