
/**
****************************(C) COPYRIGHT 2019 DJI****************************
* @file       IST8310middleware.c/h
* @brief      the file provide I2C write/read function, as the middleware of IST8310.
*             ���ļ���Ҫ�ṩI2C ��д��������ΪIST8310�������м��
* @note       IST8310 only support I2C. IST8310ֻ֧��I2C��
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
 * @brief          ��ʼ��IST8310��GPIO
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
 * @brief          ��ʼ��IST8310��ͨ�Žӿ�
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
 * @brief          ��ȡIST8310��һ���ֽ�ͨ��I2C
 * @param[in]      �Ĵ�����ַ
 * @retval         �Ĵ���ֵ
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
 * @brief          ͨ��I2Cд��һ���ֽڵ�IST8310�ļĴ�����
 * @param[in]      �Ĵ�����ַ
 * @param[in]      д��ֵ
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
 * @brief          ��ȡIST8310�Ķ���ֽ�ͨ��I2C
 * @param[in]      �Ĵ�����ʼ��ַ
 * @param[out]     ��ȡ������
 * @param[in]      ��ȡ�ֽ�����
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
 * @brief          д�����ֽڵ�IST8310�ļĴ���ͨ��I2C
 * @param[in]      �Ĵ�����ʼ��ַ
 * @param[out]     ��ȡ������
 * @param[in]      ��ȡ�ֽ�����
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
 * @brief          ��ʱx����
 * @param[in]      ms: ms����
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
 * @brief          ��ʱx΢��
 * @param[in]      us: us΢��
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
 * @brief          ����RSTN����Ϊ1
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
 * @brief          ����RSTN����Ϊ0
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
