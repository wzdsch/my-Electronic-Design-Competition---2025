#include "callbacks.h"
#include "chassis.h"

extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim13;

extern chassis_t chassis;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim == &htim14) // 10ms 100hz 底盘计算和发送
  {
    chassis_run(&chassis);
    chassis_pid_run(&chassis);
    chassis_send(&chassis);
  }
  if (htim == &htim13)
  {
    chassis_get_motor_spd(&chassis);
  }
}
