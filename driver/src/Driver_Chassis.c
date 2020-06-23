#include "Driver_Chassis.h"
#include "macro.h"
#include "config.h"
#include "handle.h"
#include "vegmath.h"
#include "math.h"

void Chassis_Init(ChassisData_Type *cd) {
    cd->vx                 = 0;
    cd->vy                 = 0;
    cd->vw                 = 0;
    cd->power              = 0;
    cd->referencePower     = 0;
    cd->lastReferencePower = 0;
    cd->targetPower        = 80;
    cd->maxPower           = 80;
    cd->maxPowerBuffer     = 60;
    cd->powerScale         = 1;
    PID_Init(&(cd->PID_Power), 1, 0, 0, 500, 10);
}

void Chassis_Update(ChassisData_Type *cd, float vx, float vy, float vw) {
    cd->vx = vx;
    cd->vy = vy;
    cd->vw = vw;
}

void Chassis_Fix(ChassisData_Type *cd, float angle) {
    float sinYaw = vegsin(angle); // 1
    float cosYaw = vegcos(angle); // 0
    float vx     = cd->vx;
    float vy     = cd->vy;
    cd->vy       = vy * cosYaw - vx * sinYaw;
    cd->vx       = vy * sinYaw + vx * cosYaw;
}

void Chassis_Calculate_Rotor_Speed(ChassisData_Type *cd) {
    float coefficient = CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE;
    cd->rotorSpeed[0] = coefficient * (cd->vy - cd->vx - cd->vw * CHASSIS_SIZE_K);
    cd->rotorSpeed[1] = coefficient * (cd->vy + cd->vx - cd->vw * CHASSIS_SIZE_K);
    cd->rotorSpeed[2] = -coefficient * (cd->vy - cd->vx + cd->vw * CHASSIS_SIZE_K);
    cd->rotorSpeed[3] = -coefficient * (cd->vy + cd->vx + cd->vw * CHASSIS_SIZE_K);
}

void Chassis_Limit_Rotor_Speed(ChassisData_Type *cd, float maxRotorSpeed) {
    float   maxSpeed = 0;
    float   scale    = 0;
    int16_t absSpeed = 0;
    uint8_t i        = 0;

    // 打擂台获得麦轮速度最大值
    for (; i < 4; i++) {
        absSpeed = ABS(cd->rotorSpeed[i]);
        if (absSpeed > maxSpeed) {
            maxSpeed = absSpeed;
        }
    }

    // 进行限幅
    if (maxSpeed > maxRotorSpeed) {
        scale = maxRotorSpeed / maxSpeed;
        Chassis_Scale_Rotor_Speed(cd, scale);
    }
}

void Chassis_Scale_Rotor_Speed(ChassisData_Type *cd, float scale) {
    cd->rotorSpeed[0] = cd->rotorSpeed[0] * scale;
    cd->rotorSpeed[1] = cd->rotorSpeed[1] * scale;
    cd->rotorSpeed[2] = cd->rotorSpeed[2] * scale;
    cd->rotorSpeed[3] = cd->rotorSpeed[3] * scale;
}

void Chassis_Limit_Power(ChassisData_Type *cd, float targetPower, float referencePower, float referencePowerBuffer, float interval) {
    PID_Type *PID_Power = &(cd->PID_Power);

    // 更新状态
    cd->targetPower    = targetPower;
    cd->referencePower = referencePower;
    cd->interval       = interval;

    // 功率拟合
    if (cd->referencePower != cd->lastReferencePower) {
        // 更新功率及缓冲
        cd->power              = cd->referencePower;
        cd->lastReferencePower = cd->referencePower;
        cd->powerBuffer        = referencePowerBuffer;
        // 重置拟合和缩放
        cd->fittingCounter = 0;
        cd->powerScale     = 1;
    } else {
        float stable;
        float ePow;
        cd->fittingCounter++;
        stable = cd->powerScale * cd->power;
        ePow   = pow(2.71828, -(cd->interval) * (1.0 / 0.04)); // 40
        // ePow      = pow(2.71828, -(cd->interval) * ((float) cd->fittingCounter) / 0.035); // 40
        cd->power = stable + (cd->power - stable) * ePow;
        // 模拟功率缓冲
        cd->powerBuffer -= (cd->referencePower - cd->maxPower) * cd->interval;
        MIAO(cd->powerBuffer, 0, cd->maxPowerBuffer);
    }

    // if (referencePowerBuffer < 30) {
    //     cd->powerBuffer = referencePowerBuffer;
    // }

    // 功率PID
    PID_Calculate(PID_Power, cd->targetPower, cd->power);
    cd->powerScale = (cd->power + PID_Power->output) / cd->power;
    MIAO(cd->powerScale, 0, 1);
    // Chassis_Scale_Rotor_Speed(cd, cd->powerScale);
}