/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Hiic.h"
#include "Pid.h"
#include "Chassis.h"
#include "Ultrasonic.h"
#include "Status.h"
#include "StepperMotor.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int wait_fun(float ms_num);
void wait_ms(float ms);
void Get_Front_dis(void);
void Get_Left_dis(void);
void Get_right_dis(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_2_STP_Pin GPIO_PIN_0
#define MOTOR_2_STP_GPIO_Port GPIOA
#define MOTOR_5_STP_Pin GPIO_PIN_1
#define MOTOR_5_STP_GPIO_Port GPIOA
#define receive_left_Pin GPIO_PIN_2
#define receive_left_GPIO_Port GPIOA
#define ultrasonic_trig_left_Pin GPIO_PIN_3
#define ultrasonic_trig_left_GPIO_Port GPIOA
#define MOTOR_3_STP_Pin GPIO_PIN_6
#define MOTOR_3_STP_GPIO_Port GPIOA
#define MOTOR_5_DIR_Pin GPIO_PIN_7
#define MOTOR_5_DIR_GPIO_Port GPIOA
#define MOTOR_DIR_1_Pin GPIO_PIN_7
#define MOTOR_DIR_1_GPIO_Port GPIOE
#define MOTOR_1_STP_Pin GPIO_PIN_9
#define MOTOR_1_STP_GPIO_Port GPIOE
#define MOTOR_DIR_4_Pin GPIO_PIN_15
#define MOTOR_DIR_4_GPIO_Port GPIOE
#define receive_right_Pin GPIO_PIN_8
#define receive_right_GPIO_Port GPIOD
#define ultrasonic_trig_right_Pin GPIO_PIN_9
#define ultrasonic_trig_right_GPIO_Port GPIOD
#define MOTOR_DIR_3_Pin GPIO_PIN_11
#define MOTOR_DIR_3_GPIO_Port GPIOD
#define MOTOR_4_STP_Pin GPIO_PIN_12
#define MOTOR_4_STP_GPIO_Port GPIOD
#define receive_front_Pin GPIO_PIN_9
#define receive_front_GPIO_Port GPIOA
#define ultrasonic_trig_front_Pin GPIO_PIN_10
#define ultrasonic_trig_front_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
enum Trace_Dir
{
  NO,
  Forward,
  Left,
  Right
};
#define CARSPEED (float)0.25
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
