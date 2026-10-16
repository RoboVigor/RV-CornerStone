#ifndef LED_APP_H
#define LED_APP_H

/* Call after CubeMX GPIO initialization. LEDs are active-low, PG1 through PG8. */
void LED_App_Init(void);
void LED_App_Step(void);

#endif
