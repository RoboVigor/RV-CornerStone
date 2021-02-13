
/**
****************************(C) COPYRIGHT 2019 DJI****************************
* @file       IST8310driver.c/h
* @brief      ist8310 is a 3-axis digital magnetometer, the file includes initialization function,
*             read magnetic field strength data function.
*             IST8310��һ���������ִ����ƣ����ļ�������ʼ����������ȡ�ų����ݺ�����
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

#include "ist8310driver.h"

// the first column:the registers of IST8310. ��һ��:IST8310�ļĴ���
// the second column: the value to be writed to the registers.�ڶ���:��Ҫд��ļĴ���ֵ
// the third column: return error value.������:���صĴ�����
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},  // enalbe interrupt  and low pin polarity.�����жϣ��������õ͵�ƽ
    {0x41, 0x09, 0x02},  // average 2 times.ƽ����������
    {0x42, 0xC0, 0x03},  // must be 0xC0. ������0xC0
    {0x0A, 0x0B, 0x04}}; // 200Hz output rate.200Hz���Ƶ��

/**
 * @brief          initialize ist8310
 * @param[in]      none
 * @retval         error value
 */
/**
 * @brief          ��ʼ��IST8310
 * @param[in]      none
 * @retval         error value
 */
uint8_t ist8310_init(void) {
    static const uint8_t wait_time = 150;
    static const uint8_t sleepTime = 50;
    uint8_t              res       = 0;
    uint8_t              writeNum  = 0;

    ist8310_GPIO_init();
    ist8310_com_init();

    ist8310_RST_L();
    ist8310_delay_ms(sleepTime);
    ist8310_RST_H();
    ist8310_delay_ms(sleepTime);

    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE) {
        return IST8310_NO_SENSOR;
    }

    // set mpu6500 sonsor config and check
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++) {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        ist8310_delay_us(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        ist8310_delay_us(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1]) {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    return IST8310_NO_ERROR;
}

/**
 * @brief          if you have read the data from STAT1 to DATAZL usaully by I2C DMA , you can use the function to solve.
 * @param[in]      status_buf:the data point from the STAT1(0x02) register of IST8310 to the DATAZL(0x08) register
 * @param[out]     ist8310_real_data:ist8310 data struct
 * @retval         none
 */
/**
 * @brief          ����Ѿ�ͨ��I2C��DMA��ʽ��ȡ���˴�STAT1��DATAZL�����ݣ�����ʹ������������д���
 * @param[in]      status_buf:����ָ��,��STAT1(0x02) �Ĵ����� DATAZL(0x08)�Ĵ���
 * @param[out]     ist8310_real_data:ist8310�����ݽṹ
 * @retval         none
 */
void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data) {

    if (status_buf[0] & 0x01) {
        int16_t temp_ist8310_data = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

        temp_ist8310_data         = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data         = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data         = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    } else {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

/**
 * @brief          read mag magnetic field strength data of IST8310 by I2C
 * @param[out]     mag variable
 * @retval         none
 */
/**
 * @brief          ͨ����ȡ�ų�����
 * @param[out]     �ų�����
 * @retval         none
 */
void ist8310_read_mag(float mag[3]) {
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    // read the "DATAXL" register (0x03)
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0]            = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1]            = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2]            = MAG_SEN * temp_ist8310_data;
}
