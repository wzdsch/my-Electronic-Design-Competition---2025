#include "pid.h"
#include "main.h"

#define LimitMax(input, max)                                                   \
  {                                                                            \
    if (input > max) {                                                         \
      input = max;                                                             \
    } else if (input < -max) {                                                 \
      input = -max;                                                            \
    }                                                                          \
  }

void PID_init(pids *pid, uint8_t mode, const fp32 PID[3], fp32 max_out,
              fp32 max_iout) {
  if (pid == NULL || PID == NULL) {
    return;
  }
  pid->mode = mode;
  pid->Kp = PID[0];
  pid->Ki = PID[1];
  pid->Kd = PID[2];
  pid->max_out = max_out;
  pid->max_iout = max_iout;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout =
      pid->Dout = pid->out = 0.0f;
}

fp32 PID_calc(pids *pid, fp32 ref, fp32 set) {
  if (pid == NULL) {
    return 0.0f;
  }
  pid->error[2] = pid->error[1];
  pid->error[1] = pid->error[0];
  pid->set = set;
  pid->fdb = ref;
  pid->error[0] = set - ref;
  switch (pid->mode) {
  case PID_POSITION:
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    break;
  case PID_DELTA:
    pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
    pid->Iout = pid->Ki * pid->error[0];
    pid->Dbuf[2] = pid->Dbuf[1];
    pid->Dbuf[1] = pid->Dbuf[0];
    pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
    pid->Dout = pid->Kd * pid->Dbuf[0];
    pid->out += pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    break;
  }
  return pid->out;
}

void PID_clear(pids *pid) {
  if (pid == NULL) {
    return;
  }
  pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
  pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
  pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
  pid->fdb = pid->set = 0.0f;
}

void pidINIT(pids *pid, uint8_t mode, fp32 KP, fp32 KI, fp32 KD, fp32 maxOut,
             fp32 maxIout) {
  fp32 val_pid[3] = {KP, KI, KD};
  PID_init(pid, mode, val_pid, maxOut, maxIout);
  PID_clear(pid);
}
