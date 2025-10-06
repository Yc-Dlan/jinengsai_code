#include "Sensor.h"
#include "main.h"
#include "StepperMotor.h"
#include "tim.h"
#include "stdlib.h"
#include "Chassis.h"
#include "Math.h"
#include "stdio.h"

extern unsigned int Sensor_Front[];
extern unsigned int Sensor_Right[];
extern unsigned int Sensor_Left[];
extern unsigned char Sensor_Color[];
extern float Move_Time[];
extern uint32_t detect_flag;
uint32_t cross_mark = 0;
uint32_t cross = 0;
extern enum Trace_Dir Trace_flag;
extern float Car_Speed;

void Get_Sensor(unsigned int *Sensor)
{
	if (Sensor == Sensor_Front)
	{
		unsigned char Digtal = IIC_Get_Digtal();
		for (int i = 0; i < 8; i++)
		{
			Sensor[i] = (Digtal >> i) & 0x01;
		}
	}
	else if (Sensor == Sensor_Right)
	{
		unsigned char Digtal = IIC_Get_Digtal2();
		for (int i = 0; i < 8; i++)
		{
			Sensor[i] = (Digtal >> i) & 0x01;
		}
	}
	else if (Sensor == Sensor_Left)
	{
		unsigned char Digtal = IIC_Get_Digtal3();
		for (int i = 0; i < 8; i++)
		{
			Sensor[i] = (Digtal >> i) & 0x01;
		}
	}
}

float Speed_Correction(void)
{
	float f = 0;
	if (Trace_flag == Forward)
	{
		if (Sensor_Front[3] == 0)
		{
			f += 0.1;
		}
		if (Sensor_Front[4] == 0)
		{
			f += -0.1;
		}
		if (Sensor_Front[2] == 0)
		{
			f += 0.2;
		}
		if (Sensor_Front[5] == 0)
		{
			f += -0.2;
		}
		if (Sensor_Front[1] == 0)
		{
			f += 0.275;
		}
		if (Sensor_Front[6] == 0)
		{
			f += -0.275;
		}
		if (Sensor_Front[0] == 0)
		{
			f += 0.35;
		}
		if (Sensor_Front[7] == 0)
		{
			f += -0.35;
		}
	}
	else if (Trace_flag == Right)
	{
		if (Sensor_Right[3] == 0)
		{
			f += 0.1;
		}
		if (Sensor_Right[4] == 0)
		{
			f += -0.1;
		}
		if (Sensor_Right[2] == 0)
		{
			f += 0.2;
		}
		if (Sensor_Right[5] == 0)
		{
			f += -0.2;
		}
		if (Sensor_Right[1] == 0)
		{
			f += 0.275;
		}
		if (Sensor_Right[6] == 0)
		{
			f += -0.275;
		}
		if (Sensor_Right[0] == 0)
		{
			f += 0.35;
		}
		if (Sensor_Right[7] == 0)
		{
			f += -0.35;
		}
	}
	else if (Trace_flag == Left)
	{
		if (Sensor_Left[3] == 0)
		{
			f += 0.1;
		}
		if (Sensor_Left[4] == 0)
		{
			f += -0.1;
		}
		if (Sensor_Left[2] == 0)
		{
			f += 0.2;
		}
		if (Sensor_Left[5] == 0)
		{
			f += -0.2;
		}
		if (Sensor_Left[1] == 0)
		{
			f += 0.275;
		}
		if (Sensor_Left[6] == 0)
		{
			f += -0.275;
		}
		if (Sensor_Left[0] == 0)
		{
			f += 0.35;
		}
		if (Sensor_Left[7] == 0)
		{
			f += -0.35;
		}
	}
	f = f*Car_Speed/0.3;
	if (fabsf(f) <= 0.0001)
	{
		f = 0;
	}
	return f;
}
// 检测前方识别到了几个路口，count为目标
int Detect_Line_F(uint32_t count)
{
	if (detect_flag == 1)
	{
		if (cross == 0 && (Sensor_Front[7] == 0) || (Sensor_Front[0] == 0))
		{
			if (Sensor_Front[7] == 0)
			{
				cross = 1;
			}
			else if (Sensor_Front[0] == 0)
			{
				cross = 2;
			}
		}
		if ((cross == 1 && (Sensor_Front[7] != 0)) || (cross == 2 && (Sensor_Front[0] != 0)))
		{
			if (cross_mark < count)
			{
				cross_mark++;
			}
			cross = 3;
		}
		if (cross == 3)
		{
			Car_Speed = 0.1;
			if (wait_fun(0.7))
			{
				Car_Speed = 0.2;
				cross = 0;
			}
		}
		if (cross_mark == count)
		{
			cross_mark = 0;
			detect_flag = 0;
			return 1;
		}
	}
	return 0;
}
// 检测左方识别到了几个路口，count为目标
int Detect_Line_L(uint32_t count)
{
	if (detect_flag == 2)
	{
		if (cross == 0 && (Sensor_Left[7] == 0) || (Sensor_Left[0] == 0))
		{
			if (Sensor_Left[7] == 0)
			{
				cross = 1;
			}
			else if (Sensor_Left[0] == 0)
			{
				cross = 2;
			}
		}
		if ((cross == 1 && (Sensor_Left[7] != 0)) || (cross == 2 && (Sensor_Left[0] != 0)))
		{
			if (cross_mark < count)
			{
				cross_mark++;
			}
			cross = 3;
		}
		if (cross == 3)
		{
			Car_Speed = 0.1;
			if (wait_fun(0.7))
			{
				Car_Speed = 0.2;
				cross = 0;
			}
		}
		if (cross_mark == count)
		{
			cross_mark = 0;
			detect_flag = 0;
			return 1;
		}
	}
	return 0;
}
// 检测右方识别到了几个路口，count为目标
int Detect_Line_R(uint32_t count)
{
	if (detect_flag == 3)
	{
		if (cross == 0 && (Sensor_Right[7] == 0) || (Sensor_Right[0] == 0))
		{
			cross = 1;
		}
		if (cross == 1 && (Sensor_Right[7] != 0) || (Sensor_Right[0] != 0))
		{
			if (cross_mark < count)
			{
				cross_mark++;
			}
			cross = 3;
		}
		if (cross_mark == count)
		{
			cross_mark = 0;
			detect_flag = 0;
			return 1;
		}
	}
	return 0;
}
// int Get_Color(void)
// {
// 	if (Sensor_Color[1] <= 110)
// 	{
// 		return 2;
// 	}
// 	else
// 	{
// 		if (Sensor_Color[0] <= 8 && Sensor_Color[0] >= 0)
// 		{
// 			return 1;
// 		}
// 		else if (Sensor_Color[0] <= 40 && Sensor_Color[0] >= 9)
// 		{
// 			return 3;
// 		}
// 	}
// 	return 1;
// }

// int Exposure(uint32_t color)
// {
// 	switch (color)
// 	{
// 	case 1:
// 		if (Sensor_Color[2] >= 110)
// 		{
// 			return 1;
// 		}
// 		else
// 		{
// 			return 0;
// 		}

// 		break;
// 	case 2:
// 		if (Sensor_Color[2] >= 110)
// 		{
// 			return 1;
// 		}
// 		else
// 		{
// 			return 0;
// 		}

// 		break;
// 	case 3:
// 		if (Sensor_Color[2] >= 110)
// 		{
// 			return 1;
// 		}
// 		else
// 		{
// 			return 0;
// 		}

// 		break;

// 	default:
// 		return 0;
// 		break;
// 	}
// }