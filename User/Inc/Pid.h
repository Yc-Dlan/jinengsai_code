#ifndef __PID_H__
#define __PID_H__

typedef struct {
   float target_value;
   float current_value;
   float err;
   float last_err;
   float last_last_err;
   float Ki,Kp,Kd;
   float output;
   float integral;
   float max_limit;
} PID_Typedef;

float GetPID(float get, float set, PID_Typedef *PID);
void PID_Init(PID_Typedef *PID, float Kp, float Ki, float Kd, float max_limit);
void Motor_pid();

#endif