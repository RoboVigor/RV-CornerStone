#include <bits/stdc++.h>

using namespace std;

uint16_t LEDXPRow   = 0;
uint16_t LEDXPState = 0;
void     LED_Run_Horse_XP() {
    LEDXPState = (LEDXPState % 11) + 1;
    if (LEDXPState <= 3)
        LEDXPRow = (LEDXPRow << 1) + 1;
    else if (LEDXPState >= 4 && LEDXPState <= 8)
        LEDXPRow = LEDXPRow << 1;
    else
        LEDXPRow = LEDXPRow - (1 << LEDXPState - 4);
    char s[8];
    itoa(LEDXPRow, s, 2);
    printf("%8s",s);
}

uint16_t LEDHorseRow   = 0;
uint16_t LEDHorseState = 0;
void     LED_Run_Horse() {
    LEDHorseState = (LEDHorseState % 26) + 1;
    if (LEDHorseState <= 8)
        LEDHorseRow = (LEDHorseRow << 1) + 1;
    else if (LEDHorseState >= 14 && LEDHorseState <= 21)
        LEDHorseRow = LEDHorseRow - (1 << LEDHorseState - 14);
    char s[8];
    itoa(LEDHorseRow, s, 2);
    printf("%2d.%8s",LEDHorseState,s);
}

int main(){
    char input;
    while(1){
        cin.get(input);
        LED_Run_Horse();
    }
    return 0;
}
