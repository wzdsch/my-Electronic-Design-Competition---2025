#include "chassis.h"
#include "motor.h"
#include "math.h"

#define SIN_60 0.86602540378443864676372317075294f
#define SIN_30 0.5f

#define MAX_SPD_SET 50.0f // 单位：编码值/10ms

/// @brief 初始化底盘结构体
/// @param chassis 底盘结构体
/// @param motor1 底盘电机指针(前)
/// @param motor2 底盘电机指针(左后)
/// @param motor3 底盘电机指针(右后)
void chassis_init(chassis_t* chassis, MG513_t* motor1, MG513_t* motor2, MG513_t* motor3)
{
  chassis->mode = DISABLE;
  chassis->motor1 = motor1;
  chassis->motor2 = motor2;
  chassis->motor3 = motor3;
  chassis->spd_x = 0;
  chassis->spd_y = 0;
  chassis->spd_z = 0;
}

/// @brief 底盘电机pid计算
/// @param chassis 
void chassis_pid_run(chassis_t* chassis) // chassis pid calc
{
  MG513_pid_run(chassis->motor1);
  MG513_pid_run(chassis->motor2);
  MG513_pid_run(chassis->motor3);
}

/// @brief 底盘电机pwm波发送
/// @param chassis 
void chassis_send(chassis_t* chassis)
{
  MG513_pwm_val_set(chassis->motor1, chassis->motor1->pid_spd.out);
  MG513_pwm_val_set(chassis->motor2, chassis->motor2->pid_spd.out);
  MG513_pwm_val_set(chassis->motor3, chassis->motor3->pid_spd.out);

  MG513_send(chassis->motor1);
  MG513_send(chassis->motor2);
  MG513_send(chassis->motor3);
}

/// @brief 底盘解算
/// @param chassis 
void chassis_run(chassis_t* chassis)
{
  switch (chassis->mode)
  {
    case DISABLE:
    {
      MG513_disable(chassis->motor1);
      MG513_disable(chassis->motor2);
      MG513_disable(chassis->motor3);

      break;
    }
    case ENABLE:
    {
      MG513_enable(chassis->motor1);
      MG513_enable(chassis->motor2);
      MG513_enable(chassis->motor3);

      fp32 v1 = chassis->spd_x + chassis->spd_z;
      fp32 v2 = (-chassis->spd_x) * SIN_30 + chassis->spd_y * SIN_60 + chassis->spd_z;
      fp32 v3 = (-chassis->spd_x) * SIN_60 - chassis->spd_y * SIN_30 + chassis->spd_z;

      fp32 scale = 1; // spd缩放

      // 获取最大转速
      fp32 max_spd = v1;
      if (v2 > max_spd)
      {
        max_spd = v2;
      }
      if (v3 > max_spd)
      {
        max_spd = v3;
      }

      if (max_spd > MAX_SPD_SET)
      {
        scale = MAX_SPD_SET / max_spd; // 已经判断过spd为正，可以直接除
        v1 *= scale;
        v2 *= scale;
        v3 *= scale;
      }

      // 电机速度设定，四舍五入
      MG513_set_target_spd(chassis->motor1, (int)(v1 >= 0 ? v1 + 0.5f : v1 - 0.5f));
      MG513_set_target_spd(chassis->motor2, (int)(v2 >= 0 ? v2 + 0.5f : v2 - 0.5f));
      MG513_set_target_spd(chassis->motor3, (int)(v3 >= 0 ? v3 + 0.5f : v3 - 0.5f));

      break;
    }
  }
}

/// @brief 获取底盘各个电机的速度(注意放在10ms 100hz的定时器中)
/// @param chassis 
void chassis_get_motor_spd(chassis_t* chassis)
{
  MG513_get_spd(chassis->motor1);
  MG513_get_spd(chassis->motor2);
  MG513_get_spd(chassis->motor3);
}
