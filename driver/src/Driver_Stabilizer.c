
#include "Driver_Stabilizer.h"
#include "Driver_CAN.h"
#include "Driver_DBUS.h"
#include "handle.h"

float Pitch_Target;
float Yaw_Target;

void CloudPara_Init(Cloud_Type *CourseAngle) {
    CourseAngle->AnglePosition = 0;
    CourseAngle->AngularSpeed  = 0;
    CourseAngle->TargetAngle   = 0;
}

void TargetAngleSet(float pitch_control, float yaw_control) {
    static float last_yaw = 0;
    Pitch_Target          = Pitch.TargetAngle + pitch_control;
    Pitch.AnglePosition   = EulerAngle.Pitch;
    Pitch.TargetAngle     = (Pitch_MaxAngle < Pitch_Target ? Pitch_MaxAngle : Pitch_Target);
    Pitch.TargetAngle     = (Pitch_MinAngle < Pitch_Target ? Pitch.TargetAngle : Pitch_MinAngle);

    if (yaw_control > 1)
        yaw_control = 1;
    else if (yaw_control < -1)
        yaw_control = -1;
    Yaw_Target = Yaw.TargetAngle + yaw_control;

    if (last_yaw - EulerAngle.Yaw > 180)
        EulerAngle.round++;
    else if (last_yaw - EulerAngle.Yaw < -180)
        EulerAngle.round--;

    last_yaw = EulerAngle.Yaw;

    EulerAngle.Yawfeedback = EulerAngle.Yaw + EulerAngle.round * 360;

    Yaw.AnglePosition = -EulerAngle.Yawfeedback;

    Yaw.TargetAngle = Yaw_Target;
}