#include "main.h"
#include "StepperMotor.h"
#include "tim.h"
#include "stdlib.h"
#include "Chassis.h"
#include "Math.h"
#include "stdio.h"
#include "Sensor.h"

extern unsigned int Sensor_Front[];
extern float Move_Time[];
extern PID_Typedef Car_pid;
extern enum Trace_Dir Trace_flag;
float correction = 0;
float V = 0;


float GetPID(float get, float set, PID_Typedef *PID)
{
	PID->target_value = set;
	PID->current_value = get;

	PID->err = PID->target_value - PID->current_value;
	PID->integral += PID->err;
	if (fabsf(PID->integral) >= 1)
	{
		PID->integral = 1;
	}

	PID->output = PID->Kp * PID->err + PID->Ki * PID->integral / 2 + PID->Kd * (PID->err - PID->last_err);

	PID->last_err = PID->err;

	if (fabsf(PID->output) >= PID->max_limit)
	{
		PID->output = PID->max_limit;
	}
	

	return PID->output;
}

void PID_Init(PID_Typedef *PID, float Kp, float Ki, float Kd, float max_limit)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->max_limit = max_limit;
	PID->integral = 0;
	PID->output = 0;
}

void Motor_pid()
{
	V = Move_Time[2] * 10.0 / 7.0;
	correction = Speed_Correction();
	if(correction != correction || fabsf(correction) <= 0.001)
	{
		correction = 0;
	}
	V += GetPID(V, V + correction, &Car_pid);
	if (Trace_flag == Forward)
	{
		Move_Time[1] = V*0.09;
	}
	if (Trace_flag == Left)
	{
		Move_Time[0] = -V*0.05;
	}
	if (Trace_flag == Right)
	{
		Move_Time[0] = V*0.05;
	}
	Move_Time[2] = V*0.7;
	if (correction == 0)
	{
		Move_Time[2] = 0;
	}
	if (Move_Time[2] >= 0.125 + Car_pid.max_limit)
	{
		Move_Time[2] = 0.125 + Car_pid.max_limit;
	}
	else if (Move_Time[2] <= -0.125 - Car_pid.max_limit)
	{
		Move_Time[2] = -0.125 - Car_pid.max_limit;
	}
}