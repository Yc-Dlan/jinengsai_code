#include "main.h"
#include "StepperMotor.h"
#include "tim.h"
#include "stdlib.h"
#include "math.h"

#define Angle_Mul 16

extern uint32_t flag[];
extern TIM_HandleTypeDef *htim[];
extern uint32_t timer[];
extern uint32_t run_flag;
extern uint32_t Go_flag;
extern uint32_t arm_num;

uint32_t target_pulses = 0;
uint32_t arm_flag = 1;
extern float arm_speed;
extern uint32_t arm_dir;

void Set_Motor_Speed(uint32_t id, float time, uint32_t dir)
{
	Motor *Motor_r = NULL;
	Motor_r = (Motor *)malloc(sizeof(Motor));
	if (time >= 0)
	{
		if (id == 1 || id == 2 || id == 4)
		{
			Motor_r->dir = 0;
		}
		else
		{
			Motor_r->dir = 1;
		}
	}
	else
	{
		if (id == 1 || id == 2 || id == 4)
		{
			Motor_r->dir = 1;
		}
		else
		{
			Motor_r->dir = 0;
		}
	}
	Motor_r->time = fabsf(time);
	Motor_r->id = id;
	flag[id - 1] = 1;

	switch (id)
	{
	case 1:
		Motor_r->htim = &htim1;
		if (Motor_r->dir == 1)
		{
			HAL_GPIO_WritePin(MOTOR_DIR_1_GPIO_PORT, MOTOR_DIR_1_PIN, GPIO_PIN_SET);
		}
		else if (Motor_r->dir == 0)
		{
			HAL_GPIO_WritePin(MOTOR_DIR_1_GPIO_PORT, MOTOR_DIR_1_PIN, GPIO_PIN_RESET);
		}
		break;
	case 2:
		Motor_r->htim = &htim2;
		if (Motor_r->dir == 1)
		{
			HAL_GPIO_WritePin(MOTOR_DIR_2_GPIO_PORT, MOTOR_DIR_2_PIN, GPIO_PIN_SET);
		}
		else if (Motor_r->dir == 0)
		{
			HAL_GPIO_WritePin(MOTOR_DIR_2_GPIO_PORT, MOTOR_DIR_2_PIN, GPIO_PIN_RESET);
		}
		break;
	case 3:
		Motor_r->htim = &htim3;
		if (Motor_r->dir == 1)
		{
			HAL_GPIO_WritePin(MOTOR_DIR_3_GPIO_PORT, MOTOR_DIR_3_PIN, GPIO_PIN_SET);
		}
		else if (Motor_r->dir == 0)
		{
			HAL_GPIO_WritePin(MOTOR_DIR_3_GPIO_PORT, MOTOR_DIR_3_PIN, GPIO_PIN_RESET);
		}
		break;
	case 4:
		Motor_r->htim = &htim4;
		if (Motor_r->dir == 1)
		{
			HAL_GPIO_WritePin(MOTOR_DIR_4_GPIO_PORT, MOTOR_DIR_4_PIN, GPIO_PIN_RESET);
		}
		else if (Motor_r->dir == 0)
		{
			HAL_GPIO_WritePin(MOTOR_DIR_4_GPIO_PORT, MOTOR_DIR_4_PIN, GPIO_PIN_SET);
		}
		break;
	default:
		break;
	}

	float arr;
	arr = 312.5f * Motor_r->time;
	if (arr == 0)
	{
		__HAL_TIM_SET_COUNTER(Motor_r->htim, 0);
		__HAL_TIM_SET_COMPARE(Motor_r->htim, TIM_CHANNEL_1, 0);
	}
	else
	{
		__HAL_TIM_SET_COUNTER(Motor_r->htim, 0);
		__HAL_TIM_SET_AUTORELOAD(Motor_r->htim, arr);
		__HAL_TIM_SET_COMPARE(Motor_r->htim, TIM_CHANNEL_1, arr / 2);
	}
	free(Motor_r);
	Motor_r = NULL;
}

void Set_Angle(float angle)
{

	if (arm_flag == 1)
	{
		// 设置方向
		uint32_t dir = (angle >= 0) ? 1 : 0;
		GPIO_PinState pin_state = (dir == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;
		HAL_GPIO_WritePin(MOTOR_DIR_5_GPIO_PORT, MOTOR_DIR_5_PIN, pin_state);

		// 计算需要的脉冲数（假设1.8度/脉冲，16细分）
		// 取绝对值，因为方向已单独处理
		target_pulses = (uint32_t)(fabsf(angle) / 1.8f * 16.0f);

		// 计算定时器参数（根据你的具体硬件配置调整）
		// 假设这是计算脉冲频率的参数
		float arr = 312.5f * 0.08f * 3.1415f / 0.3f * 2;

		// 配置定时器
		__HAL_TIM_SET_COUNTER(&htim5, 0);
		__HAL_TIM_SET_AUTORELOAD(&htim5, (uint32_t)arr);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint32_t)(arr / 2.0f));

		if (arm_num >= target_pulses)
		{
			arm_flag = 0;
			__HAL_TIM_SET_COUNTER(&htim5, 0);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
			arm_num = 0;
		}
	}
}

// void Set_Arm_Speed(float time, uint32_t dir)
//{
//	if(dir == 1)
//		{
//			HAL_GPIO_WritePin(MOTOR_DIR_ARM_GPIO_PORT, MOTOR_DIR_ARM_PIN, GPIO_PIN_SET);
//		}
//		else if(dir == 0)
//		{
//			HAL_GPIO_WritePin(MOTOR_DIR_ARM_GPIO_PORT, MOTOR_DIR_ARM_PIN, GPIO_PIN_RESET);
//		}
//	float arr;
//	arr = 312.5f * time / 16;
//	if(arr == 0)
//	{
//		__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
//	}
//	else
//	{
//		__HAL_TIM_SET_AUTORELOAD(&htim12, arr);
//	    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr/2);
//	}
// }

// void Arm(void)
//{
//     if (arm_flag == 1)
//     {
//         Set_Arm_Speed(arm_speed, arm_dir);
//     }
//     else
//     {
//         Set_Arm_Speed(0, arm_dir);
//     }
// }
//  void Set_Motor_Pos(uint32_t *id, float *time, float Pos, uint32_t *dir)
//  {
//  	Set_Motor_Speed(id[0], time[0], dir[0]);
//  	Set_Motor_Speed(id[1], time[1], dir[1]);
//  	Set_Motor_Speed(id[2], time[2], dir[2]);
//  	Set_Motor_Speed(id[3], time[3], dir[3]);

// 	if(timer[0] >= Pos)
// 	{
// 		uint32_t i;
// 		for(i = 0;i <= 3;i++)
// 		{
// 				__HAL_TIM_SET_COMPARE(htim[id[i]-1], TIM_CHANNEL_1, 0);
// 			flag[id[i]-1] = 0;
// 	    timer[id[i]-1] = 0;
// 		}
// 		run_flag = 0;
// 		Go_flag = 0;
//     Back_flag = 0;
//     Right_flag = 0;
//     Left_flag = 0;
//     Rotation_flag = 0;
// 	}
// }
