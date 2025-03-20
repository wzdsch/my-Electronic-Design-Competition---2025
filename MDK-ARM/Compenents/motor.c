#include "motor.h"

/// @brief 电机结构体初始化
/// @param MG513 电机结构体指针
/// @param htim 电机pwm波所用的定时器
/// @param pos_channel 正传通道
/// @param nag_channel 反转通道
void MG513_init(MG513_t *MG513, TIM_HandleTypeDef *tx_htim, uint32_t tx_pos_channel, \
uint32_t tx_nag_channel, TIM_HandleTypeDef *rx_htim, uint32_t rx_channel_A, \
uint32_t rx_channel_B, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout)
{
  MG513->real_spd = 0;
  MG513->target_spd = 0;
  MG513->en_flg = 0;
  MG513->pwm_val = 0;
  MG513->tx_htim = tx_htim;
  MG513->tx_pos_channel = tx_pos_channel;
  MG513->tx_nag_channel = tx_nag_channel;
  MG513->rx_htim = rx_htim;
  MG513->rx_channel_A = rx_channel_A;
  MG513->rx_channel_B = rx_channel_B;

  pidINIT(&(MG513->pid_spd), PID_POSITION, kp, ki, kd, max_out, max_iout);
}

/// @brief 使能电机
/// @param MG513 电机结构体
void MG513_enable(MG513_t* MG513)
{
  MG513->en_flg = 1;
}

/// @brief 失能电机
/// @param MG513 电机结构体
void MG513_disable(MG513_t* MG513)
{
  MG513->en_flg = 0;
}

/// @brief 设定电机pwm占空比(失能置0)
/// @param MG513 
/// @param pwm_val 
void MG513_pwm_val_set(MG513_t* MG513, int pwm_val)
{
  if (MG513->en_flg == DISABLE)
  {
    MG513->pwm_val = 0;
  }
  else
  {
    MG513->pwm_val = pwm_val;
  }
}

/// @brief 设定目标转速
/// @param MG513 
/// @param spd 
void MG513_set_target_spd(MG513_t* MG513, int spd)
{
  MG513->target_spd = spd;
}

/// @brief 电机pid计算
/// @param MG513 
void MG513_pid_run(MG513_t* MG513)
{
  PID_calc(&(MG513->pid_spd), MG513->real_spd, MG513->target_spd);
}

/// @brief 判断是否使能，并发送pwm波
/// @param MG513 
void MG513_send(MG513_t* MG513)
{
  if (MG513->en_flg == ENABLE)
  {
    if (MG513->target_spd >= 0)
    {  
      __HAL_TIM_SetCompare(MG513->tx_htim, MG513->tx_pos_channel, MG513->pwm_val);
      __HAL_TIM_SetCompare(MG513->tx_htim, MG513->tx_nag_channel, 0);
    }
    else if (MG513->target_spd < 0)
    {
      __HAL_TIM_SetCompare(MG513->tx_htim, MG513->tx_nag_channel, MG513->pwm_val);
      __HAL_TIM_SetCompare(MG513->tx_htim, MG513->tx_pos_channel, 0);
    }
  }
  else
  {
    __HAL_TIM_SetCompare(MG513->tx_htim, MG513->tx_nag_channel, 0);
    __HAL_TIM_SetCompare(MG513->tx_htim, MG513->tx_pos_channel, 0);
  }
}

/// @brief 获取电机速度(必须放在10ms 100hz的定时器中！！！)
/// @param MG513 
void MG513_get_spd(MG513_t* MG513)
{
  MG513->real_spd = (short)__HAL_TIM_GET_COUNTER(MG513->rx_htim);
    __HAL_TIM_GET_COUNTER(MG513->rx_htim) = 0;
}
