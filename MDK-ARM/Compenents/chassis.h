#ifndef CHASSIS_H
#define CHASSIS_H

#include "motor.h"

typedef struct
{
  int mode; // 状态  0. DISABLE 1. ENABLE

  MG513_t *motor1; // 前方电机
  MG513_t *motor2; // 左后方电机
  MG513_t *motor3; // 右后方电机

  int spd_x; // 左右速度，右为正
  int spd_y; // 前后速度，前为正
  int spd_z; // 旋转速度，逆时针为正
}chassis_t;

// enum chassis_mode
// {
//   DISABLE = 0,
//   ENABLE = 1,
// };

void chassis_init(chassis_t *chassis, MG513_t *motor1, MG513_t *motor2, MG513_t *motor3);
void chassis_pid_run(chassis_t *chassis);
void chassis_send(chassis_t *chassis);
void chassis_run(chassis_t *chassis);
void chassis_get_motor_spd(chassis_t *chassis);

#endif
