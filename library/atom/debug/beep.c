#include "beep.h"

void Beep_Init(void) {
    // BSP_Beep_Init()
}

const uint16_t tone_tab[] = {
    3822, 3405, 3033, 2863, 2551, 2272, 2024, // bass 1~7
    1911, 1702, 1526, 1431, 1275, 1136, 1012, // mid 1~7
    955,  851,  758,  715,  637,  568,  506,  // treble 1~7
};

const Sound_Tone_Type Music_Scope_XP[Music_Len_XP] = {
    Mi3H, Mi3H, Mi3M, Do1H, Do1H, Do1H, So5M, So5M, So5M, Mi3H, Mi3H, Do1H, Do1H, Do1H, Do1H, Do1H, Do1H, Silent};

const Sound_Tone_Type Music_Scope_Earth[Music_Len_Earth] = {
    Silent, Silent, Silent, Silent, Silent, Silent, Silent, Silent, Silent, Silent, Silent, Silent, Mi3M,   Mi3M,   So5M,   So5M, La6M, La6M,   Silent,
    Silent, Silent, Silent, So5M,   So5M,   La6M,   La6M,   Silent, Silent, Silent, Silent, So5M,   So5M,   La6M,   La6M,   Do1H, Do1H, So5M,   So5M,
    La6M,   La6M,   Mi3M,   Mi3M,   Silent, Silent, Mi3M,   Mi3M,   So5M,   So5M,   La6M,   La6M,   Mi3H,   Mi3H,   Do1H,   Do1H, Re2H, Re2H,   La6M,
    La6M,   Silent, Silent, Mi3M,   Mi3M,   So5M,   So5M,   La6M,   La6M,   Silent, Silent, Silent, Silent, So5M,   So5M,   La6M, La6M, Silent, Silent,
    Silent, Silent, So5M,   So5M,   La6M,   La6M,   Do1H,   Do1H,   So5M,   So5M,   La6M,   La6M,   Mi3M,   Mi3M,   So5M,   So5M, Do1M, Do1M,   Re2M,
    Re2M,   Mi3M,   Mi3M,   Mi3M,   Mi3M,   Do1H,   Do1H,   Do1H,   Do1H,   La6M,   La6M,   La6M,   La6M,   Mi3H,   Mi3H,   Mi3H, Mi3H, Re2H,   Re2H,
    Mi3H,   Re2H,   Do1H,   Do1H,   Re2H,   Re2H,   La6M,   La6M,   La6M,   La6M,   Silent, Silent, Silent, Silent, La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   La6M,   La6M,   Silent, La6M,   Do1H,   Re2H,   Mi3M,   La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   So5M,   So5M,   Silent, So5M,   So5M,   La6M,   La6M,   La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   La6M,   La6M,   Silent, La6M,   Do1H,   Re2H,   Mi3M,   La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   Fa4H,   Fa4H,   Silent, Fa4H,   Fa4H,   Mi3H,   Mi3H,   La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   La6M,   La6M,   Silent, La6M,   Do1H,   Re2H,   Mi3M,   La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   So5M,   So5M,   Silent, So5M,   So5M,   La6M,   La6M,   Silent, La6M, La6M, La6M,   La6M,
    Silent, La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   La6M,   La6M,   Silent, La6M,   Do1H,   Re2H,   Mi3M,   La6H, La6H, La6H,   La6H,
    La6H,   La6H,   So5M,   So5M,   Silent, So5M,   So5M,   La6M,   La6M,   Silent, La6M,   La6M,   La6M,   La6M,   Silent};

const Sound_Tone_Type Music_Scope_Sky[Music_Len_Sky] = {
    La6M, Si7M,   Do1H,   Do1H, Do1H,   Si7M,   Do1H,   Do1H,   Mi3H, Mi3H,   Si7M,   Si7M,   Si7M,  Si7M, Silent, Mi3M,   Mi3M,   La6M,   La6M,
    La6M, So5M,   La6M,   La6M, Do1H,   Do1H,   So5M,   So5M,   So5M, So5M,   Silent, Mi3M,   Mi3M,  Fa4M, Fa4M,   Fa4M,   Mi3M,   Fa4M,   Fa4M,
    Do1H, Do1H,   Mi3M,   Mi3M, Mi3M,   Mi3M,   Silent, Silent, Do1H, Do1H,   Do1H,   Si7M,   Si7M,  Fa4M, Fa4M,   Fa4M,   Si7M,   Si7M,   Silent,
    Si7M, Si7M,   Si7M,   Si7M, Silent, Silent, Silent, La6M,   Si7M, Do1H,   Do1H,   Do1H,   Si7M,  Do1H, Do1H,   Mi3H,   Mi3H,   Si7M,   Si7M,
    Si7M, Si7M,   Silent, Mi3M, Mi3M,   La6M,   La6M,   La6M,   So5M, La6M,   La6M,   Do1H,   Do1H,  So5M, So5M,   So5M,   So5M,   Silent, Re2M,
    Mi3M, Fa4M,   Fa4M,   Do1H, Si7M,   Si7M,   Do1H,   Do1H,   Re2H, Re2H,   Mi3H,   Do1H,   Do1H,  Do1H, Do1H,   Silent, Do1H,   Si7M,   La6M,
    La6M, Si7M,   Si7M,   So5M, So5M,   La6M,   La6M,   La6M,   La6M, Silent, Do1H,   Re2H,   Mi3H,  Mi3H, Mi3H,   Re2H,   Mi3H,   Mi3H,   So5H,
    So5H, Re2H,   Re2H,   Re2H, Re2H,   Silent, So5M,   So5M,   Do1H, Do1H,   Do1H,   Si7M,   Do1H,  Do1H, Mi3H,   Mi3H,   Mi3H,   Mi3H,   Mi3H,
    Mi3H, Silent, La6M,   Si7M, Do1H,   Do1H,   Si7M,   Si7M,   Re2H, Re2H,   Do1H,   Do1H,   Do1H,  So5M, So5M,   So5M,   So5M,   Silent, Fa4H,
    Fa4H, Mi3H,   Mi3H,   Re2H, Re2H,   Do1H,   Do1H,   Mi3H,   Mi3H, Mi3H,   Mi3H,   Silent, Mi3H,  Mi3H, La6H,   La6H,   La6H,   So5H,   So5H,
    So5H, Mi3H,   Re2H,   Do1H, Do1H,   Do1H,   Silent, Silent, Do1H, Re2H,   Re2H,   Do1H,   Re2H,  Re2H, Re2H,   So5H,   So5H,   Mi3H,   Mi3H,
    Mi3H, Mi3H,   Silent, Mi3H, Mi3H,   La6H,   La6H,   La6H,   So5H, So5H,   So5H,   Mi3H,   Re2H,  Do1H, Do1H,   Do1H,   Silent, Silent, Do1H,
    Re2H, Re2H,   Do1H,   Re2H, Re2H,   Re2H,   Si7M,   Si7M,   La6M, La6M,   La6M,   La6M,   Silent};

const Sound_Tone_Type Music_Scope_Soul[Music_Len_Soul] = {
    Mi3M, Mi3M, Mi3M, Mi3M, So5M,   So5M, So5M, So5M,   La6M, La6M, La6M, La6M, La6M, La6M,   La6M, La6M, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M,  Mi3M, Mi3M,   Mi3M,
    Re2M, Re2M, Re2M, Re2M, Do1M,   Do1M, Do1M, Do1M,   Re2M, Re2M, Re2M, Re2M, Mi3M, Mi3M,   Mi3M, Mi3M, So5M, So5M, So5M, So5M, So5M,  So5M, So5M,   So5M,
    Mi3M, Mi3M, Mi3M, Mi3M, Mi3M,   Mi3M, Mi3M, Mi3M,   Re2M, Re2M, Re2M, Re2M, Do1M, Do1M,   Do1M, Do1M, Re2M, Re2M, Re2M, Re2M, Mi3M,  Mi3M, Mi3M,   Mi3M,
    Do1M, Do1M, Do1M, Do1M, Silent, Do1M, Do1M, Do1M,   Do1M, Mi3M, Mi3M, Mi3M, Mi3M, Silent, Mi3M, Mi3M, Mi3M, Mi3M, Do1M, Do1M, Do1M,  Do1M, Silent, Do1M,
    Do1M, Do1M, Do1M, Mi3M, Mi3M,   Mi3M, Mi3M, Silent, Mi3M, Mi3M, Mi3M, Mi3M, Do1M, Do1M,   Do1M, Do1M, Do1M, Do1M, Do1M, Do1M, Silent};

const Sound_Tone_Type Music_Scope_Bird[Music_Len_Bird]={Silent};

void Sing(Sound_Tone_Type tone) {
    if (Silent == tone)
        BEEP_CH = 0;
    else {
        BEEP_ARR = tone_tab[tone];
        BEEP_CH  = tone_tab[tone] / 2;
    }
}
