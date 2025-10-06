#ifndef __STEPPERMOTOR_H__
#define __STEPPERMOTOR_H__
#include "main.h"

#define MOTOR_DIR_1_GPIO_PORT GPIOE
#define MOTOR_DIR_1_PIN GPIO_PIN_7
#define MOTOR_Stp_1_GPIO_PORT GPIOE
#define MOTOR_Stp_1_PIN GPIO_PIN_9

#define MOTOR_DIR_2_GPIO_PORT GPIOC  
#define MOTOR_DIR_2_PIN GPIO_PIN_5
#define MOTOR_Stp_2_GPIO_PORT GPIOA
#define MOTOR_Stp_2_PIN GPIO_PIN_0

#define MOTOR_DIR_3_GPIO_PORT GPIOD
#define MOTOR_DIR_3_PIN GPIO_PIN_11
#define MOTOR_Stp_3_GPIO_PORT GPIOA
#define MOTOR_Stp_3_PIN GPIO_PIN_6

#define MOTOR_DIR_4_GPIO_PORT GPIOE
#define MOTOR_DIR_4_PIN GPIO_PIN_15
#define MOTOR_Stp_4_GPIO_PORT GPIOD
#define MOTOR_Stp_4_PIN GPIO_PIN_12

#define MOTOR_DIR_5_GPIO_PORT GPIOA
#define MOTOR_DIR_5_PIN GPIO_PIN_7
#define MOTOR_Stp_5_GPIO_PORT GPIOA
#define MOTOR_Stp_5_PIN GPIO_PIN_1

#define MOTOR_DIR_ARM_GPIO_PORT GPIOB
#define MOTOR_DIR_ARM_PIN GPIO_PIN_15
#define MOTOR_Stp_ARM_GPIO_PORT GPIOB
#define MOTOR_Stp_ARM_PIN GPIO_PIN_14

// #define MOTOR_En_GPIO_PORT GPIOA
// #define MOTOR_En_PIN GPIO_PIN_5


typedef struct
{
    uint32_t id;
    uint32_t dir;
    float time;
    TIM_HandleTypeDef *htim;
}Motor;


// Motor_Group *a = (Motor_Group *)malloc(sizeof(Motor_Group))

void Set_Motor_Speed(uint32_t id, float time, uint32_t dir);
void Set_Speed(float time, uint32_t dir);
void Set_Motor_Pos(uint32_t *id, float *time, float Pos);
void Set_Arm_Speed(float time, uint32_t dir);
void Arm(void);
// void Set_Motor_Pos(float time,uint32_t pos,uint32_t dir);

#endif