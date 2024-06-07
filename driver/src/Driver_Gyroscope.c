#define __DRIVER_GYROSCOPE_GLOBALS
#include "math.h"
#include "Driver_Filter.h"
#include "config.h"
#include "MadgwickAHRS.h"
#include "Driver_Gyroscope.h"

static float          rollAngle;
static float          pitchAngle;
static float          yawAngle;
static float          xSpeed;
static float          ySpeed;
static float          zSpeed;
static float          xAcc;
static float          yAcc;
static float          zAcc;
static float          xMag;
static float          yMag;
static float          zMag;
extern volatile float beta;
static int16_t        debug_pitch = 0;

double yawoffset_add=0;

Filter_Type Filter_Yaw = {.count = 0, .thresholdLB = GYROSCOPE_YAW_FILTER_THRESHOLD};

extern ImuData_Type ImuData;

void Gyroscope_Init(GyroscopeData_Type *GyroscopeData, uint16_t startupDelay) {
    GyroscopeData->startupCounter = 0;
    GyroscopeData->modification = 0;
#ifdef STM32F427_437xx
    MPU6500_Initialize();
    MPU6500_EnableInt();
#endif
#ifdef STM32F40_41xxx
    while (BMI088_init()) {
    }
    ist8310_init();
#endif
    if (startupDelay != 0) {
#ifdef STM32F427_437xx
        beta = 5;
        while (1) {
            LED_Set_Progress(GyroscopeData->startupCounter / (startupDelay / 7) + 1);
            if (GyroscopeData->startupCounter >= startupDelay) {
                beta = 0.1;
                break;
            }
        }
#endif
#ifdef STM32F40_41xxx
        beta = 5;
        while (1) {
            LED_Set_Colour(GyroscopeData->startupCounter / startupDelay * 255, 0, 0);
            if (GyroscopeData->startupCounter >= startupDelay) {
                beta = 0.5;
                break;
            }
        }
#endif
    }
}

// MPU6500数据读取,成功返回1  失败返回0
int Gyroscope_Update(GyroscopeData_Type *GyroscopeData) {
#ifdef STM32F427_437xx
    static uint8_t mpu_buf[20];

    //尝试读取数据
    if (IIC_ReadData(MPU_IIC_ADDR, MPU6500_ACCEL_XOUT_H, mpu_buf, 14) == 0xff) return 0;

    //成功的话进行赋值
    ImuData.temp = (((int16_t) mpu_buf[6]) << 8) | mpu_buf[7];
#if BOARD_FRONT_IS_UP
    ImuData.az = (((int16_t) mpu_buf[4]) << 8) | mpu_buf[5];
    ImuData.gz = ((((int16_t) mpu_buf[12]) << 8) | mpu_buf[13]) - ImuData.gz_bias;
#else
    ImuData.az = -1 * (((int16_t) mpu_buf[4]) << 8) | mpu_buf[5];
    ImuData.gz = -1 * ((((int16_t) mpu_buf[12]) << 8) | mpu_buf[13]) - ImuData.gz_bias;
#endif
#if BOARD_SHORT_SIDE_IS_PARALLEL_TO_PITCH
    ImuData.ax = (((int16_t) mpu_buf[0]) << 8) | mpu_buf[1];
    ImuData.ay = (((int16_t) mpu_buf[2]) << 8) | mpu_buf[3];
    ImuData.gx = ((((int16_t) mpu_buf[8]) << 8) | mpu_buf[9]) - ImuData.gx_bias;
    ImuData.gy = ((((int16_t) mpu_buf[10]) << 8) | mpu_buf[11]) - ImuData.gy_bias;
#else
    ImuData.ay = (((int16_t) mpu_buf[0]) << 8) | mpu_buf[1];
    ImuData.ax = (((int16_t) mpu_buf[2]) << 8) | mpu_buf[3];
    ImuData.gy = ((((int16_t) mpu_buf[8]) << 8) | mpu_buf[9]) - ImuData.gy_bias;
    ImuData.gx = ((((int16_t) mpu_buf[10]) << 8) | mpu_buf[11]) - ImuData.gx_bias;
#endif
#endif
#ifdef STM32F40_41xxx
    static uint8_t buf[8];

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    ImuData.ax = (int16_t)((buf[1] << 8) | buf[0]);
    ImuData.ay = (int16_t)((buf[3] << 8) | buf[2]);
    ImuData.az = (int16_t)((buf[5] << 8) | buf[4]);

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        ImuData.gx = (int16_t)((buf[3] << 8) | buf[2]);
        ImuData.gy = (int16_t)((buf[5] << 8) | buf[4]);
        ImuData.gz = (int16_t)((buf[7] << 8) | buf[6]);
    }

    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    if ((int16_t)((buf[0] << 3) | (buf[1] >> 5)) > 1023) {
        ImuData.temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5)) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    } else {
        ImuData.temp = ((int16_t)((buf[0] << 3) | (buf[1] >> 5)) - 2048) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }

    ist8310_IIC_read_muli_reg(0x03, buf, 6);
    ImuData.mx = (int16_t)((buf[1] << 8) | buf[0]);
    ImuData.my = (int16_t)((buf[3] << 8) | buf[2]);
    ImuData.mz = (int16_t)((buf[5] << 8) | buf[4]);
#endif

    // 读取完成进行解算
    Gyroscope_Solve(GyroscopeData);

    // 返回成功值
    return 1;
}

void Gyroscope_Solve(GyroscopeData_Type *GyroscopeData) {
    xSpeed = (float) ((ImuData.gx / GYROSCOPE_LSB) * PI / 180.0);
    ySpeed = (float) ((ImuData.gy / GYROSCOPE_LSB) * PI / 180.0);
    zSpeed = (float) ((ImuData.gz / GYROSCOPE_LSB) * PI / 180.0);
    xAcc   = (float) (ImuData.ax / ACCELERATE_LSB);
    yAcc   = (float) (ImuData.ay / ACCELERATE_LSB);
    zAcc   = (float) (ImuData.az / ACCELERATE_LSB);
	
	
#ifdef STM32F40_41xxx
    xMag = (float) (ImuData.mx / MAGNETIC_LSB);
    yMag = (float) (ImuData.my / MAGNETIC_LSB);
    zMag = (float) (ImuData.mz / MAGNETIC_LSB);
#endif

    // GD算法或Madgwick算法,梯度算法,网上开源
#ifdef STM32F427_437xx
    MadgwickAHRSupdateIMU(xSpeed, ySpeed, zSpeed, xAcc, yAcc, zAcc);
#endif
#ifdef STM32F40_41xxx
    MadgwickAHRSupdate(xSpeed, ySpeed, zSpeed, xAcc, yAcc, zAcc, xMag, yMag, zMag);
#endif

    // 四元数->欧拉角
    pitchAngle = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180 / PI;
    rollAngle  = asin(2.0f * (q0 * q2 - q1 * q3)) * 180 / PI;
    yawAngle   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180 / PI;

    // 更新滤波器
    Filter_Update(&Filter_Yaw, yawAngle);

    // 计算连续 Yaw 角
    if (Filter_Yaw.diff > 300) {
        Filter_Yaw.offset -= 360;
    } else if (Filter_Yaw.diff < -300) {
        Filter_Yaw.offset += 360;
    }

    //前馈
    GyroscopeData->modification += GYROSCOPE_YAW_MODIFICATION;

    // 应用滤波
    GyroscopeData->yaw = Filter_Apply_Limit_Breadth(&Filter_Yaw) + GyroscopeData->yawoffset+GyroscopeData->modification;

    // 输出欧拉角
    if (GyroscopeData->startupCounter == GYROSCOPE_START_UP_DELAY - 1) {
        GyroscopeData->yawoffset = -GyroscopeData->yaw;
    }

    GyroscopeData->pitch = -pitchAngle;
    GyroscopeData->roll  = rollAngle;
    debug_pitch          = GyroscopeData->pitch;

    // 输出欧拉角
#if GYROSCOPE_START_UP_DELAY_ENABLED
    if (GyroscopeData->startupCounter < GYROSCOPE_START_UP_DELAY) {
        GyroscopeData->startupCounter += 1;
    }
#endif
}

float Gyroscope_Get_Filter_Diff(void) {
    return Filter_Yaw.diff;
}

void Gyroscope_Set_Bias(ImuData_Type *ImuData, int16_t gx_bias, int16_t gy_bias, int16_t gz_bias) {
    ImuData->gx_bias = gx_bias;
    ImuData->gy_bias = gy_bias;
    ImuData->gz_bias = gz_bias;
}
