#include "main.h"
#include "StepperMotor.h"
#include "tim.h"
#include "stdlib.h"
#include "Chassis.h"
#include "Math.h"
#include "stdio.h"
#include "Pid.h"

extern uint32_t flag[];
extern TIM_HandleTypeDef *htim[];
extern uint32_t timer[];
extern uint32_t Stop_flag[];
extern uint32_t Stop_All_flag;
extern float Move_Time[];
extern uint32_t Rotation_flag;
extern enum Trace_Dir Trace_flag;
extern uint32_t Left_flag;
extern uint32_t Right_flag;
extern uint32_t Back_flag;
extern uint32_t Forward_flag;
extern uint32_t Forward_dis;
extern uint32_t Left_dis;
extern uint32_t Right_dis;
extern uint32_t Back_dis;
extern float Rotation_Degree;
extern float Car_Speed;

uint32_t id[] = {1, 2, 3, 4};

#define H 0.143
#define W 0.195

float Motor_V[] = {0, 0, 0, 0, 0};

void Go(float Vx, float Vy, float Vw)
{
	uint32_t i;
	if (Vx + Vy - Vw * (H / 2 + W / 2) == 0)
	{
		Motor_V[0] = 0;
	}
	else
	{
		Motor_V[0] = 0.08 * 3.1415 / (Vx + Vy - Vw * (H / 2 + W / 2));
	}

	if (Vx - Vy - Vw * (H / 2 + W / 2) == 0)
	{
		Motor_V[1] = 0;
	}
	else
	{
		Motor_V[1] = 0.08 * 3.1415 / (Vx - Vy - Vw * (H / 2 + W / 2));
	}

	if (Vx - Vy + Vw * (H / 2 + W / 2) == 0)
	{
		Motor_V[2] = 0;
	}
	else
	{
		Motor_V[2] = 0.08 * 3.1415 / (Vx - Vy + Vw * (H / 2 + W / 2));
	}

	if (Vx + Vy + Vw * (H / 2 + W / 2) == 0)
	{
		Motor_V[3] = 0;
	}
	else
	{
		Motor_V[3] = 0.08 * 3.1415 / (Vx + Vy + Vw * (H / 2 + W / 2));
	}
	for (i = 0; i <= 3; i++)
	{
		Set_Motor_Speed(i + 1, Motor_V[i], 1);
	}
}

void Stop(uint32_t id)
{
	__HAL_TIM_SET_COMPARE(htim[id - 1], TIM_CHANNEL_1, 0);
	flag[id - 1] = 0;
	timer[id - 1] = 0;
	Stop_flag[id - 1] = 0;
}

void Stop_All()
{
	if (Stop_All_flag == 1)
	{
		Move_Time[0] -= 0.05 * Move_Time[0];
		Move_Time[1] -= 0.05 * Move_Time[1];
		Move_Time[2] = 0;
		if (Move_Time[0] <= 0.01)
		{
			Move_Time[0] = 0;
		}
		if (Move_Time[1] <= 0.01)
		{
			Move_Time[1] = 0;
		}
		Trace_flag = NO;
		if (Move_Time[0] == 0 && Move_Time[1] == 0)
		{
			Stop_All_flag = 0;
		}
		Go(Move_Time[0], Move_Time[1], Move_Time[2]);
	}
}
// 开始循迹函数，Trace_flag表示循迹方向
void Start_Trace()
{
	if (Trace_flag == NO)
	{
	}
	else
	{
		if (Trace_flag == Forward)
		{
			if (Move_Time[0] < Car_Speed)
			{
				Move_Time[0] += 0.03 * (Car_Speed - Move_Time[0]);
			}
			else
			{
				Move_Time[0] = Car_Speed;
			}
			Motor_pid();
			Go(Move_Time[0], Move_Time[1], Move_Time[2]);
		}
		if (Trace_flag == Left)
		{
			if (Move_Time[1] < Car_Speed)
			{
				Move_Time[1] += 0.03 * (Car_Speed - Move_Time[0]);
			}
			else
			{
				Move_Time[1] = Car_Speed;
			}
			Motor_pid();
			Go(Move_Time[0], Move_Time[1], Move_Time[2]);
		}
		if (Trace_flag == Right)
		{
			if (Move_Time[1] > -Car_Speed)
			{
				Move_Time[1] += 0.05 * (-Car_Speed - Move_Time[0]);
			}
			else
			{
				Move_Time[1] = -Car_Speed;
			}
			Motor_pid();
			Go(Move_Time[0], Move_Time[1], Move_Time[2]);
		}
	}
}
// 旋转固定角度，0为逆时针，1为顺时针
void Rotation_Deg(float Degree, uint32_t dir)
{
	if (Rotation_flag == 1)
	{
		if (dir == 0)
		{
			Rotation_Degree = Degree;
			Move_Time[0] = 0;
			Move_Time[1] = 0;
			Move_Time[2] = 2;
			Go(Move_Time[0], Move_Time[1], Move_Time[2]);
		}
		else
		{
			Rotation_Degree = Degree;
			Move_Time[0] = 0;
			Move_Time[1] = 0;
			Move_Time[2] = -2;
			Go(Move_Time[0], Move_Time[1], Move_Time[2]);
		}
	}
}
// 向后走固定距离
void Get_Back(uint32_t dis)
{
	if (Back_flag == 1)
	{
		Back_dis = dis;
		Move_Time[0] = -0.3;
		Move_Time[1] = 0;
		Move_Time[2] = 0;
		Go(Move_Time[0], Move_Time[1], Move_Time[2]);
	}
}
// 向右走固定距离
void Go_Right(uint32_t dis)
{
	if (Right_flag == 1)
	{
		Right_dis = dis;
		Move_Time[0] = 0;
		Move_Time[1] = -0.3;
		Move_Time[2] = 0;
		Go(Move_Time[0], Move_Time[1], Move_Time[2]);
	}
}
// 向左走固定距离
void Go_Left(uint32_t dis)
{
	if (Left_flag == 1)
	{
		Left_dis = dis;
		Move_Time[0] = 0;
		Move_Time[1] = 0.3;
		Move_Time[2] = 0;
		Go(Move_Time[0], Move_Time[1], Move_Time[2]);
	}
}
// 向前走固定距离
void Go_Forward(uint32_t dis)
{
	if (Forward_flag == 1)
	{
		Forward_dis = dis;
		Move_Time[0] = 0.3;
		Move_Time[1] = 0;
		Move_Time[2] = 0;
		Go(Move_Time[0], Move_Time[1], Move_Time[2]);
	}
}