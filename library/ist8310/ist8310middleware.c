
/**
****************************(C) COPYRIGHT 2019 DJI****************************
* @file       IST8310middleware.c/h
* @brief      the file provide I2C write/read function, as the middleware of IST8310.
*             本文件主要提供I2C 读写函数，作为IST8310驱动的中间件
* @note       IST8310 only support I2C. IST8310只支持I2C。
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Dec-26-2018     RM              1. done
*
@verbatim
==============================================================================

==============================================================================
@endverbatim
****************************(C) COPYRIGHT 2019 DJI****************************
*/

#include "ist8310middleware.h"
#include "mpu6500_i2c.h"

#define IIC_SCL_H() PAout(8) = 1
#define IIC_SCL_L() PAout(8) = 0
#define IIC_SDA_H() PCout(9) = 1
#define IIC_SDA_L() PCout(9) = 0
#define IIC_SDA_Read() PCin(9)

/**
 * @brief          initialize ist8310 gpio.
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          初始化IST8310的GPIO
 * @param[in]      none
 * @retval         none
 */
void ist8310_GPIO_init(void) {
}

/**
 * @brief          initialize ist8310 communication interface
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          初始化IST8310的通信接口
 * @param[in]      none
 * @retval         none
 */
void ist8310_com_init(void) {
}

/**
 * @brief          read a byte of ist8310 by i2c
 * @param[in]      register address
 * @retval         value of the register
 */
/**
 * @brief          读取IST8310的一个字节通过I2C
 * @param[in]      寄存器地址
 * @retval         寄存器值
 */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg) {
    uint8_t res = 0;
    IIC_ReadData(IST8310_IIC_ADDRESS, reg, &res, 1);
    return res;
}

/**
 * @brief          write a byte of ist8310 by i2c
 * @param[in]      register address
 * @param[in]      write value
 * @retval         value of the register
 */
/**
 * @brief          通过I2C写入一个字节到IST8310的寄存器中
 * @param[in]      寄存器地址
 * @param[in]      写入值
 * @retval         none
 */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data) {
    IIC_WriteData(IST8310_IIC_ADDRESS, reg, data);
}

/**
 * @brief          read multiple byte of ist8310 by i2c
 * @param[in]      register start address
 * @param[out]     read buffer
 * @param[in]      Size Amount of data to be read
 * @retval         none
 */
/**
 * @brief          读取IST8310的多个字节通过I2C
 * @param[in]      寄存器开始地址
 * @param[out]     存取缓冲区
 * @param[in]      读取字节总数
 * @retval         none
 */
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
    IIC_ReadData(IST8310_IIC_ADDRESS, reg, buf, len);
}

/**
 * @brief          write multiple byte of ist8310 by i2c
 * @param[in]      register address
 * @param[out]     write buffer
 * @param[in]      Size Amount of data to be sent
 * @retval         none
 */
/**
 * @brief          写入多个字节到IST8310的寄存器通过I2C
 * @param[in]      寄存器开始地址
 * @param[out]     存取缓冲区
 * @param[in]      读取字节总数
 * @retval         none
 */
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len) {
    while (len != 0) {
        IIC_WriteData(IST8310_IIC_ADDRESS, reg, *data);
        data++;
        len--;
    }
}

/**
 * @brief          delay x millisecond
 * @param[in]      ms: ms millisecond
 * @retval         none
 */
/**
 * @brief          延时x毫秒
 * @param[in]      ms: ms毫秒
 * @retval         none
 */
void ist8310_delay_ms(uint16_t ms) {
    while (ms--) {
        ist8310_delay_us(1000);
    }
}

/**
 * @brief          delay x microsecond
 * @param[in]      us: us microsecond
 * @retval         none
 */
/**
 * @brief          延时x微秒
 * @param[in]      us: us微秒
 * @retval         none
 */
void ist8310_delay_us(uint16_t us) {
    uint32_t ticks = 0;
    uint32_t told = 0, tnow = 0, tcnt = 0;
    uint32_t reload = 0;
    reload          = SysTick->LOAD;
    ticks           = us * 72;
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

/**
 * @brief          set the RSTN PIN to 1
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          设置RSTN引脚为1
 * @param[in]      none
 * @retval         none
 */
void ist8310_RST_H(void) {
    GPIO_SetBits(GPIOG, GPIO_Pin_6);
}

/**
 * @brief          set the RSTN PIN to 0
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          设置RSTN引脚为0
 * @param[in]      none
 * @retval         none
 */
void ist8310_RST_L(void) {
    GPIO_ResetBits(GPIOG, GPIO_Pin_6);
}

void ist8310_IIC_Write(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint8_t Data) {
    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    I2C_GenerateSTART(I2Cx, ENABLE);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {
    }
    I2C_Send7bitAddress(I2Cx, DevAddress, I2C_Direction_Transmitter);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    }
    I2C_SendData(I2Cx, MemAddress);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
    }
    I2C_SendData(I2Cx, Data);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
    }
    I2C_GenerateSTOP(I2Cx, ENABLE);
}

uint8_t ist8310_IIC_Read(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress) {
    uint8_t Data;

    I2C_AcknowledgeConfig(I2Cx, ENABLE);
    I2C_GenerateSTART(I2Cx, ENABLE);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {
    }
    I2C_Send7bitAddress(I2Cx, DevAddress, I2C_Direction_Transmitter);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
    }
    I2C_SendData(I2Cx, MemAddress);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING)) {
    }
    I2C_GenerateSTART(I2Cx, ENABLE);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {
    }
    I2C_Send7bitAddress(I2Cx, DevAddress, I2C_Direction_Receiver);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
    }
    I2C_AcknowledgeConfig(I2Cx, DISABLE);

    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
    }
    Data = I2C_ReceiveData(I2Cx);

    I2C_GenerateSTOP(I2Cx, ENABLE);

    return Data;
}
