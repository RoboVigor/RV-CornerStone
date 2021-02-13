#include <bits/stdc++.h>

// clang-format off

using namespace std;

#define PERIPH_BASE           ((uint32_t)0x40000000)
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

#define RCC_APB2Periph_TIM1              ((uint32_t)0x00000001)
#define RCC_APB1Periph_TIM2              ((uint32_t)0x00000001)
#define RCC_APB2Periph_TIM8              ((uint32_t)0x00000002)
#define RCC_APB1Periph_TIM4              ((uint32_t)0x00000004)
#define RCC_APB1Periph_TIM5              ((uint32_t)0x00000008)
#define RCC_APB2Periph_TIM10             ((uint32_t)0x00020000)

#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000)
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400)

#define GPIO_AF_TIM1          ((uint8_t)0x01)
#define GPIO_AF_TIM2          ((uint8_t)0x01)
#define GPIO_AF_TIM8          ((uint8_t)0x03)
#define GPIO_AF_TIM4          ((uint8_t)0x02)
#define GPIO_AF_TIM5          ((uint8_t)0x02)
#define GPIO_AF_TIM10         ((uint8_t)0x03)


// copy from struct definition of TIM_TypeDef
#define CCR1 0x34
#define CCR2 0x38
#define CCR3 0x3C
#define CCR4 0x40


#define RCC_AHB1Periph_GPIOA             ((uint32_t)0x00000001)
#define RCC_AHB1Periph_GPIOC             ((uint32_t)0x00000004)
#define RCC_AHB1Periph_GPIOD             ((uint32_t)0x00000008)
#define RCC_AHB1Periph_GPIOE             ((uint32_t)0x00000010)
#define RCC_AHB1Periph_GPIOF             ((uint32_t)0x00000020)
#define RCC_AHB1Periph_GPIOH             ((uint32_t)0x00000080)
#define RCC_AHB1Periph_GPIOI             ((uint32_t)0x00000100)

#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000)

#define GPIO_PinSource0            ((uint8_t)0x00)
#define GPIO_PinSource1            ((uint8_t)0x01)
#define GPIO_PinSource2            ((uint8_t)0x02)
#define GPIO_PinSource3            ((uint8_t)0x03)
#define GPIO_PinSource4            ((uint8_t)0x04)
#define GPIO_PinSource5            ((uint8_t)0x05)
#define GPIO_PinSource6            ((uint8_t)0x06)
#define GPIO_PinSource7            ((uint8_t)0x07)
#define GPIO_PinSource8            ((uint8_t)0x08)
#define GPIO_PinSource9            ((uint8_t)0x09)
#define GPIO_PinSource10           ((uint8_t)0x0A)
#define GPIO_PinSource11           ((uint8_t)0x0B)
#define GPIO_PinSource12           ((uint8_t)0x0C)
#define GPIO_PinSource13           ((uint8_t)0x0D)
#define GPIO_PinSource14           ((uint8_t)0x0E)
#define GPIO_PinSource15           ((uint8_t)0x0F)

#define GPIO_Pin_0                 ((uint16_t)0x0001)  /* Pin 0 selected */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /* Pin 1 selected */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /* Pin 2 selected */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /* Pin 3 selected */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /* Pin 4 selected */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /* Pin 5 selected */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /* Pin 6 selected */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /* Pin 7 selected */
#define GPIO_Pin_8                 ((uint16_t)0x0100)  /* Pin 8 selected */
#define GPIO_Pin_9                 ((uint16_t)0x0200)  /* Pin 9 selected */
#define GPIO_Pin_10                ((uint16_t)0x0400)  /* Pin 10 selected */
#define GPIO_Pin_11                ((uint16_t)0x0800)  /* Pin 11 selected */
#define GPIO_Pin_12                ((uint16_t)0x1000)  /* Pin 12 selected */
#define GPIO_Pin_13                ((uint16_t)0x2000)  /* Pin 13 selected */
#define GPIO_Pin_14                ((uint16_t)0x4000)  /* Pin 14 selected */
#define GPIO_Pin_15                ((uint16_t)0x8000)  /* Pin 15 selected */
#define GPIO_Pin_All               ((uint16_t)0xFFFF)  /* All pins selected */


typedef struct {
    uint32_t      RCC_APBxPeriph_TIMx;
    uint32_t      TIMx_BASE;
    uint8_t       GPIO_AF_TIMx;
    uint8_t       Channel;
    uint32_t      GPIOx_BASE;
    uint32_t      GPIO_PinSourcex;
    uint32_t      RCC_AHB1Periph_GPIOx;
    uint16_t      GPIO_Pin_x;
    uint8_t       CCRx;
    // GPIO_TypeDef *GPIOx;
    // TIM_TypeDef * TIMx;
} PWM_Type;



/*
    * Range                    emit    len   position
    * RCC_APBxPeriph_TIM[x]    >>0     4     <<28
    * APB[x]PERIPH_BASE        >>16    4     <<24
    * TIM[x]_BASE              >>8     4     <<20
    * GPIO_AF_TIM[x]           >>0     4     <<16
    * TIMx_Channel[x]          >>0     4     <<12
    * GPIO[x]_BASE             >>8     8     <<4
    * GPIO_PinSource[x]        >>0     4     <<0
    */
    // uint32_t PWM_PD12 = ((RCC_APB1Periph_TIM4&0x0F)<<28)+(((TIM4_BASE>>16)&0x0F)<<24)+(((TIM4_BASE>>8)&0x0F)<<20)+(GPIO_AF_TIM4<<16)+(1<<12)+(((GPIOD_BASE>>8)&0xFF)<<4)+(GPIO_PinSource12&0x0F));

uint32_t generatePWMPortCode(PWM_Type *PWMx){
    uint32_t portCode = 0;
    portCode += (PWMx->RCC_APBxPeriph_TIMx &0x0F) <<28;
    portCode += ((PWMx->TIMx_BASE >>16) &0x0F) <<24;
    portCode += ((PWMx->TIMx_BASE >>8) &0x0F) <<20;
    portCode += PWMx->GPIO_AF_TIMx <<16;
    portCode += PWMx->Channel <<12;
    portCode += ((PWMx->GPIOx_BASE >>8) &0xFF) <<4;
    portCode += PWMx->GPIO_PinSourcex &0x0F;
    return portCode;
}

void BSP_PWM_Set_Port(PWM_Type *PWMx, uint32_t PWM_Px) {
    PWMx->RCC_APBxPeriph_TIMx = PWM_Px >>28;
    PWMx->TIMx_BASE = PERIPH_BASE + ((PWM_Px >>24 &0x0F) <<16) + ((PWM_Px >>20 &0x0F) <<8);
    PWMx->GPIO_AF_TIMx = PWM_Px >>16 &0xF;
    PWMx->Channel = PWM_Px >>12 &0xF;
    PWMx->GPIOx_BASE = AHB1PERIPH_BASE + ((PWM_Px >>4 &0xFF) <<8);
    PWMx->GPIO_PinSourcex = PWM_Px &0xF;
    PWMx->RCC_AHB1Periph_GPIOx = 1 <<((PWM_Px >>4 &0xFF) /4);
    PWMx->GPIO_Pin_x           = 1 <<(PWM_Px & 0x0F);
    PWMx->CCRx                 = PWMx->Channel*4 + 0x30;
    // PWMx->GPIOx             = (GPIO_TypeDef *) PWMx->GPIOx_Base;
    // PWMx->TIMx              = (TIM_TypeDef *) PWMx->TIMx_BASE;
}

void testPortCode(string portName, PWM_Type *PWMx, uint32_t PWM_Px) {
    cout << "Testing " << portName << ": ";
    PWM_Type PWM_Test_struct;
    PWM_Type * PWM_Test = &PWM_Test_struct;
    BSP_PWM_Set_Port(PWM_Test, PWM_Px);
    if(PWM_Test->RCC_APBxPeriph_TIMx!=PWMx->RCC_APBxPeriph_TIMx) cout << "ERR RCC_APBxPeriph_TIMx\n";
    else if(PWM_Test->TIMx_BASE!=PWMx->TIMx_BASE) cout << "ERR TIMx_BASE\n";
    else if(PWM_Test->GPIO_AF_TIMx!=PWMx->GPIO_AF_TIMx) cout << "ERR GPIO_AF_TIMx\n";
    else if(PWM_Test->Channel!=PWMx->Channel) cout << "ERR Channel\n";
    else if(PWM_Test->GPIOx_BASE!=PWMx->GPIOx_BASE) cout << "ERR GPIOx_BASE\n";
    else if(PWM_Test->GPIO_PinSourcex!=PWMx->GPIO_PinSourcex) cout << "ERR GPIO_PinSourcex\n";
    else if(PWM_Test->RCC_AHB1Periph_GPIOx!=PWMx->RCC_AHB1Periph_GPIOx) cout << "ERR RCC_AHB1Periph_GPIOx\n";
    else if(PWM_Test->GPIO_Pin_x!=PWMx->GPIO_Pin_x) cout << "ERR GPIO_Pin_x\n";
    else if(PWM_Test->CCRx!=PWMx->CCRx) cout << "ERR CCRx\n";
    else cout << "PASS\n";
}


int main(){

    string names[26] = {"PWM_PD12","PWM_PD13","PWM_PD14","PWM_PD15","PWM_PH10","PWM_PH11","PWM_PH12","PWM_PI0","PWM_PA0","PWM_PA1","PWM_PA2","PWM_PA3","PWM_PI5","PWM_PI6","PWM_PI7","PWM_PI2","PWM_PA8","PWM_PA9","PWM_PA10","PWM_PA11","PWM_PE9","PWM_PE11","PWM_PE13","PWM_PE14","PWM_PC6","PWM_PF6"};
    uint32_t codes[26];
    PWM_Type PWM_PORTS[26] = {
        //A
        {RCC_APB1Periph_TIM4, TIM4_BASE, GPIO_AF_TIM4, 1, GPIOD_BASE, GPIO_PinSource12, RCC_AHB1Periph_GPIOD, GPIO_Pin_12, CCR1},
        {RCC_APB1Periph_TIM4, TIM4_BASE, GPIO_AF_TIM4, 2, GPIOD_BASE, GPIO_PinSource13, RCC_AHB1Periph_GPIOD, GPIO_Pin_13, CCR2},
        {RCC_APB1Periph_TIM4, TIM4_BASE, GPIO_AF_TIM4, 3, GPIOD_BASE, GPIO_PinSource14, RCC_AHB1Periph_GPIOD, GPIO_Pin_14, CCR3},
        {RCC_APB1Periph_TIM4, TIM4_BASE, GPIO_AF_TIM4, 4, GPIOD_BASE, GPIO_PinSource15, RCC_AHB1Periph_GPIOD, GPIO_Pin_15, CCR4},
        {RCC_APB1Periph_TIM5, TIM5_BASE, GPIO_AF_TIM5, 1, GPIOH_BASE, GPIO_PinSource10, RCC_AHB1Periph_GPIOH, GPIO_Pin_10, CCR1},
        {RCC_APB1Periph_TIM5, TIM5_BASE, GPIO_AF_TIM5, 2, GPIOH_BASE, GPIO_PinSource11, RCC_AHB1Periph_GPIOH, GPIO_Pin_11, CCR2},
        {RCC_APB1Periph_TIM5, TIM5_BASE, GPIO_AF_TIM5, 3, GPIOH_BASE, GPIO_PinSource12, RCC_AHB1Periph_GPIOH, GPIO_Pin_12, CCR3},
        {RCC_APB1Periph_TIM5, TIM5_BASE, GPIO_AF_TIM5, 4, GPIOI_BASE, GPIO_PinSource0, RCC_AHB1Periph_GPIOI, GPIO_Pin_0, CCR4},

        {RCC_APB1Periph_TIM2, TIM2_BASE, GPIO_AF_TIM2, 1, GPIOA_BASE, GPIO_PinSource0, RCC_AHB1Periph_GPIOA, GPIO_Pin_0, CCR1},
        {RCC_APB1Periph_TIM2, TIM2_BASE, GPIO_AF_TIM2, 2, GPIOA_BASE, GPIO_PinSource1, RCC_AHB1Periph_GPIOA, GPIO_Pin_1, CCR2},
        {RCC_APB1Periph_TIM2, TIM2_BASE, GPIO_AF_TIM2, 3, GPIOA_BASE, GPIO_PinSource2, RCC_AHB1Periph_GPIOA, GPIO_Pin_2, CCR3},
        {RCC_APB1Periph_TIM2, TIM2_BASE, GPIO_AF_TIM2, 4, GPIOA_BASE, GPIO_PinSource3, RCC_AHB1Periph_GPIOA, GPIO_Pin_3, CCR4},
        {RCC_APB2Periph_TIM8, TIM8_BASE, GPIO_AF_TIM8, 1, GPIOI_BASE, GPIO_PinSource5, RCC_AHB1Periph_GPIOI, GPIO_Pin_5, CCR1},
        {RCC_APB2Periph_TIM8, TIM8_BASE, GPIO_AF_TIM8, 2, GPIOI_BASE, GPIO_PinSource6, RCC_AHB1Periph_GPIOI, GPIO_Pin_6, CCR2},
        {RCC_APB2Periph_TIM8, TIM8_BASE, GPIO_AF_TIM8, 3, GPIOI_BASE, GPIO_PinSource7, RCC_AHB1Periph_GPIOI, GPIO_Pin_7, CCR3},
        {RCC_APB2Periph_TIM8, TIM8_BASE, GPIO_AF_TIM8, 4, GPIOI_BASE, GPIO_PinSource2, RCC_AHB1Periph_GPIOI, GPIO_Pin_2, CCR4},

        //B
        {RCC_APB2Periph_TIM1, TIM1_BASE, GPIO_AF_TIM1, 1, GPIOA_BASE, GPIO_PinSource8, RCC_AHB1Periph_GPIOA, GPIO_Pin_8, CCR1},
        {RCC_APB2Periph_TIM1, TIM1_BASE, GPIO_AF_TIM1, 2, GPIOA_BASE, GPIO_PinSource9, RCC_AHB1Periph_GPIOA, GPIO_Pin_9, CCR2},
        {RCC_APB2Periph_TIM1, TIM1_BASE, GPIO_AF_TIM1, 3, GPIOA_BASE, GPIO_PinSource10, RCC_AHB1Periph_GPIOA, GPIO_Pin_10, CCR3},
        {RCC_APB2Periph_TIM1, TIM1_BASE, GPIO_AF_TIM1, 4, GPIOA_BASE, GPIO_PinSource11, RCC_AHB1Periph_GPIOA, GPIO_Pin_11, CCR4},

        //C
        {RCC_APB2Periph_TIM1, TIM1_BASE, GPIO_AF_TIM1, 1, GPIOE_BASE, GPIO_PinSource9, RCC_AHB1Periph_GPIOE, GPIO_Pin_9, CCR1},
        {RCC_APB2Periph_TIM1, TIM1_BASE, GPIO_AF_TIM1, 2, GPIOE_BASE, GPIO_PinSource11, RCC_AHB1Periph_GPIOE, GPIO_Pin_11, CCR2},
        {RCC_APB2Periph_TIM1, TIM1_BASE, GPIO_AF_TIM1, 3, GPIOE_BASE, GPIO_PinSource13, RCC_AHB1Periph_GPIOE, GPIO_Pin_13, CCR3},
        {RCC_APB2Periph_TIM1, TIM1_BASE, GPIO_AF_TIM1, 4, GPIOE_BASE, GPIO_PinSource14, RCC_AHB1Periph_GPIOE, GPIO_Pin_14, CCR4},
        {RCC_APB2Periph_TIM8, TIM8_BASE, GPIO_AF_TIM8, 1, GPIOC_BASE, GPIO_PinSource6, RCC_AHB1Periph_GPIOC, GPIO_Pin_6, CCR1},
        {RCC_APB2Periph_TIM10, TIM10_BASE, GPIO_AF_TIM10, 1, GPIOF_BASE, GPIO_PinSource6, RCC_AHB1Periph_GPIOF, GPIO_Pin_6, CCR1},
    };

    int i;
    //generate port code
    for(i=0; i<26; i++){
        codes[i] = generatePWMPortCode(&(PWM_PORTS[i]));
    }
    //test port code
    for(i=0; i<26; i++){
        testPortCode(names[i], &(PWM_PORTS[i]), codes[i]);
    }
    //print port code
    for(i=0; i<26; i++){
        printf("#define %s 0x%08x\n", names[i].c_str(), codes[i]);
    }

     return 0;
}


