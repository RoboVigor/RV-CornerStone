#define __DRIVER_MPU6500_GLOBALS

#include "mpu6500_driver.h"
#include "Driver_Gyroscope.h"
#include "config.h"

extern ImuData_Type ImuData;
int16_t             IMU_id;

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
                                                                        // the INT pin emits a 50us long pulse, not latched    bypass mode
                                                                        // enabled
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
    if (IIC_WriteData(MPU_IIC_ADDR, MPU6500_SMPLRT_DIV, GYROSCOPE_FREQ) == 0xff) // Sample Rate: Gyro output rate / (1 + 1) = 500Hz
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
        delay_ms(5);
    }
}
