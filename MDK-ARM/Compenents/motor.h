#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"

typedef struct motor
{
  int en_flg; // 0:disable 1:enable
  int pwm_val; // 0 ~ 200
  int target_spd; // 目标速度
  int real_spd; // 反馈速度

  pids pid_spd;

  TIM_HandleTypeDef *tx_htim; // 发送pwm定时器
  TIM_HandleTypeDef *rx_htim; // 接收pwm定时器

  uint32_t rx_channel_A; // A相接收通道
  uint32_t rx_channel_B; // B相接收通道

  uint32_t tx_pos_channel; // 发送正转通道
  uint32_t tx_nag_channel; // 发送反转通道
}MG513_t;

void MG513_init(MG513_t *MG513, TIM_HandleTypeDef *tx_htim, uint32_t tx_pos_channel,
                uint32_t tx_nag_channel, TIM_HandleTypeDef *rx_tim, uint32_t rx_channel_A,
                uint32_t rx_channel_B, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout);
void MG513_enable(MG513_t *MG513);
void MG513_disable(MG513_t *MG513);
void MG513_pwm_val_set(MG513_t *MG513, int pwm_val);
void MG513_set_target_spd(MG513_t *MG513, int spd);
void MG513_pid_run(MG513_t *MG513);
void MG513_send(MG513_t *MG513);
void MG513_get_spd(MG513_t *MG513);

#endif
