#define __DRIVER_MPU6500_GLOBALS

#include "delay.h"
#include "mpu6500_driver.h"
#include "mpu6500_i2c.h"
#include "mpu6500_interrupt.h"
#include "MPU6500_IST8310.h"

// MPU6050 初始化,成功返回0  失败返回 0xff
int MPU6500_Init(void) {
    unsigned char temp_data = 0x00;

    IIC_GPIO_Init(); //初始化IIC接口

    if (IIC_ReadData(MPU_IIC_ADDR, MPU6500_WHO_AM_I, &temp_data, 1) == 0) //确定IIC总线上挂接的是否是MPU6050
    {
        if (temp_data != MPU6500_ID)
            return 0xff; //校验失败,返回0xff
        else
            IMU_id = temp_data;
    } else {
        return 0xff; //读取失败 返回0xff
    }

    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_PWR_MGMT_1, 0x01) == 0xff) //解除休眠状态
    {
        return 0xff;
    }
    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_CONFIG, 0x01) == 0xff) // Digital Low-Pass Filter:DLPF_CFG is 3, Fs is 1khz
    {                                                              // acc bandwidth 44Hz,gyro 42Hz
        return 0xff;
    }
    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_GYRO_CONFIG, 0x18) == 0xff) // FS_SEL 3 : gyroscope full scale range is +-2000degs/s
    {
        return 0xff;
    }
    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_ACCEL_CONFIG, 0x10) == 0xff) // AFS_SEL 1: accelerometer full scale range is +-8g
    {
        return 0xff;
    }
    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_ACCEL_CONFIG_2, 0x01) == 0xff) //
    {
        return 0xff;
    }
    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_INT_PIN_CFG, 0x02) == 0xff) // logic level for the INT pin is active high
                                                                        // the INT pin emits a 50us long pulse, not latched    bypass mode enabled
    {
        return 0xff;
    }
    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_INT_ENABLE, 0x00) == 0xff) // disable data ready interrupt
    {
        return 0xff;
    }

    //设置mpu6050 IIC masters mode  disabled 不让mpu6050控制aux IIC接口
    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_USER_CTRL, 0x00) == 0xff) // disable data ready interrupt
    {
        return 0xff;
    }

    //设置IIC masters mode 为 bypass mode enabled,在INT_PIN_CFG中配置
    delay_ms(500);
    return 0;
}

int MPU6500_EnableInt(void) {
    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_SMPLRT_DIV, 0x01) == 0xff) // Sample Rate: Gyro output rate / (1 + 1) = 500Hz
    {
        return 0xff;
    }

    delay_ms(10);
    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_INT_ENABLE, 0x01) == 0xff) // enable data ready interrupt
    {
        return 0xff;
    }

    return 0;
}

void MPU6500_Initialize(void) {
    while (MPU6500_Init() == 0xff) {
        delay_ms(200);
    }
}

// MPU6500  数据读取,成功返回0  失败返回 0xff

int MPU6500_ReadData(uint8_t Slave_Addr, uint8_t Reg_Addr, uint8_t *Data, uint8_t Num) {
    // IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14)
    if (IIC_ReadData(Slave_Addr, Reg_Addr, Data, Num) == 0xff) return 0xff;

    return 0;
}

void MPU6500_getMotion6(void) {
    MPU6500_ReadData(MPU_IIC_ADDR, MPU6500_ACCEL_XOUT_H, mpu_buf, 14);
    mpu6500_data.ax   = (((int16_t) mpu_buf[0]) << 8) | mpu_buf[1];
    mpu6500_data.ay   = (((int16_t) mpu_buf[2]) << 8) | mpu_buf[3];
    mpu6500_data.az   = (((int16_t) mpu_buf[4]) << 8) | mpu_buf[5];
    mpu6500_data.temp = (((int16_t) mpu_buf[6]) << 8) | mpu_buf[7];
    mpu6500_data.gx   = (((int16_t) mpu_buf[8]) << 8) | mpu_buf[9];
    mpu6500_data.gx += mpu6500_data.gx_offset;
    mpu6500_data.gy = (((int16_t) mpu_buf[10]) << 8) | mpu_buf[11];
    mpu6500_data.gy += mpu6500_data.gy_offset;
    mpu6500_data.gz = (((int16_t) mpu_buf[12]) << 8) | mpu_buf[13];
    mpu6500_data.gz += mpu6500_data.gz_offset;
}
