#include "BMI088middleware.h"

void BMI088_GPIO_init(void) {
}

void BMI088_com_init(void) {
}

void BMI088_delay_ms(uint16_t ms) {
    while (ms--) {
        BMI088_delay_us(1000);
    }
}

void BMI088_delay_us(uint16_t us) {

    uint32_t ticks  = 0;
    uint32_t told   = 0;
    uint32_t tnow   = 0;
    uint32_t tcnt   = 0;
    uint32_t reload = 0;
    reload          = SysTick->LOAD;
    ticks           = us * 168;
    told            = SysTick->VAL;
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            if (tnow < told) {
                tcnt += told - tnow;
            } else {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) {
                break;
            }
        }
    }
}

void BMI088_ACCEL_NS_L(void) {
    GPIO_ResetBits(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);
}
void BMI088_ACCEL_NS_H(void) {
    GPIO_SetBits(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);
}

void BMI088_GYRO_NS_L(void) {
    GPIO_ResetBits(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
}
void BMI088_GYRO_NS_H(void) {
    GPIO_SetBits(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
}

uint8_t BMI088_read_write_byte(uint8_t txdata) {
    uint8_t rx_data;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
    }                               //等待发送区空
    SPI_I2S_SendData(SPI1, txdata); //通过外设 SPIx 发送一个 byte 数据
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
    }                                    //等待接收完
    rx_data = SPI_I2S_ReceiveData(SPI1); //返回通过 SPIx 最近接收的数据
    return rx_data;
}
