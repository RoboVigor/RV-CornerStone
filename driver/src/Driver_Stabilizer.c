#include "Driver_Stabilizer.h"
#include "Driver_CAN.h"
#include "Driver_DBUS.h"
#include "handle.h"
#include "macro.h"

float pitch_target;
float yaw_target;

void Stabilizer_Init_Course_Angle(Stabilizer_Type *CourseAngle) {
  CourseAngle->anglePosition = 0;
  CourseAngle->angularSpeed = 0;
  CourseAngle->targetAngle = 0;
}

void Stabilizer_Set_TargetAngle(float pitch_control, float yaw_control) {
  static float s_last_yaw = 0;
  // if (pitch_control > 0.5) {
  //   Pitch_Angle.targetAngle += 0.05;
  // } else if (pitch_control < -0.5) {
  //   Pitch_Angle.targetAngle -= 0.05;
  // };
  if (yaw_control > 0.5) {
    Yaw_Angle.targetAngle += 0.8;
  } else if (yaw_control < -0.5) {
    Yaw_Angle.targetAngle -= 0.8;
  };
  Pitch_Angle.anglePosition = EulerAngle.	Pitch;
  MIAO(yaw_control, -1, 1);
  Pitch_Angle.targetAngle = Pitch_Angle.targetAngle + pitch_control;
  // yaw_target = Yaw_Angle.targetAngle + yaw_control;
  Yaw_Angle.anglePosition = -EulerAngle.Yaw;
  // Yaw_Angle.targetAngle = yaw_target;
}