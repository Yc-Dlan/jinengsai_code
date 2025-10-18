#include "Ultrasonic.h"
#include "string.h"

#define CPU_FREQUENCY_MHZ (int)(HAL_RCC_GetHCLKFreq() / 1000000)

volatile uint8_t Obs_index_front = 0;
volatile uint8_t Obs_index_left = 0;
volatile uint8_t Obs_index_right = 0;

volatile float Obs_distance_front = 0;
volatile float Obs_distance_left = 0;
volatile float Obs_distance_right = 0;
volatile uint32_t Obs_buff_front = 0;
volatile uint32_t Obs_buff_left = 0;
volatile uint32_t Obs_buff_right = 0;
volatile uint32_t Obs_data_ready = 0;
volatile uint32_t Obs_index = 0;

uint32_t pulse_time_us = 0;
uint32_t distance_cm = 0;

void Ultrasonic_Init(void)
{
    Obs_data_ready = 0;
    Obs_index_front = 0;
    Obs_index_left = 0;
    Obs_index_right = 0;
    Obs_index = 0;
    Obs_buff_front = 0;
    Obs_buff_left = 0;
    Obs_buff_right = 0;
}

void Ultrasonic_Trig_front(void)
{
    Obs_data_ready = 0;

    // 正确的触发时序：低电平→高电平(10us)→低电平
    HAL_GPIO_WritePin(ultrasonic_trig_front_GPIO_Port, ultrasonic_trig_front_Pin, GPIO_PIN_RESET);

    // 使用循环实现微秒级延迟（根据你的CPU频率调整）
    for (volatile int i = 0; i < 30; i++)
        ; // 约2us延迟

    HAL_GPIO_WritePin(ultrasonic_trig_front_GPIO_Port, ultrasonic_trig_front_Pin, GPIO_PIN_SET);

    for (volatile int i = 0; i < 100; i++)
    {
        __NOP();
    } // 约10us延迟

    HAL_GPIO_WritePin(ultrasonic_trig_front_GPIO_Port, ultrasonic_trig_front_Pin, GPIO_PIN_RESET);
}

void Ultrasonic_Trig_left(void)
{
    Obs_data_ready = 0;

    // 正确的触发时序：低电平→高电平(10us)→低电平
    HAL_GPIO_WritePin(ultrasonic_trig_left_GPIO_Port, ultrasonic_trig_left_Pin, GPIO_PIN_RESET);

    // 使用循环实现微秒级延迟（根据你的CPU频率调整）
    for (volatile int i = 0; i < 30; i++)
        ; // 约2us延迟

    HAL_GPIO_WritePin(ultrasonic_trig_left_GPIO_Port, ultrasonic_trig_left_Pin, GPIO_PIN_SET);

    for (volatile int i = 0; i < 100; i++)
    {
        __NOP();
    } // 约10us延迟

    HAL_GPIO_WritePin(ultrasonic_trig_left_GPIO_Port, ultrasonic_trig_left_Pin, GPIO_PIN_RESET);
}

void Ultrasonic_Trig_right(void)
{
    Obs_data_ready = 0;

    // 正确的触发时序：低电平→高电平(10us)→低电平
    HAL_GPIO_WritePin(ultrasonic_trig_right_GPIO_Port, ultrasonic_trig_right_Pin, GPIO_PIN_RESET);

    // 使用循环实现微秒级延迟（根据你的CPU频率调整）
    for (volatile int i = 0; i < 30; i++)
        ; // 约2us延迟

    HAL_GPIO_WritePin(ultrasonic_trig_right_GPIO_Port, ultrasonic_trig_right_Pin, GPIO_PIN_SET);

    for (volatile int i = 0; i < 100; i++)
    {
        __NOP();
    } // 约10us延迟

    HAL_GPIO_WritePin(ultrasonic_trig_right_GPIO_Port, ultrasonic_trig_right_Pin, GPIO_PIN_RESET);
}


// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//     if (huart->Instance == USART1) {
//         if (++Obs_index_front >= 3) {
//             // 完成一帧数据接收
//             if ((Obs_buff_front[0] + Obs_buff_front[1]) == Obs_buff_front[2]) {
//                 Obs_distance_front = (Obs_buff_front[0] << 8) | Obs_buff_front[1];
//                 Obs_data_ready = 1;
//             } else {
//                 Obs_distance_front = 0xFFFF;
//             }
//             Obs_index_front = 0;
//         }

//         // 继续接收下一字节
//         HAL_UART_Receive_IT(&huart1, (uint8_t*)&Obs_buff_front[Obs_index_front], 1);
//     }

//     if (huart->Instance == USART2) {
//         if (++Obs_index_left >= 3) {
//             // 完成一帧数据接收
//             if ((Obs_buff_left[0] + Obs_buff_left[1]) == Obs_buff_left[2]) {
//                 Obs_distance_left = (Obs_buff_left[0] << 8) | Obs_buff_left[1];
//                 Obs_data_ready = 1;
//             } else {
//                 Obs_distance_left = 0xFFFF;
//             }
//             Obs_index_left = 0;
//         }

//         // 继续接收下一字节
//         HAL_UART_Receive_IT(&huart2, (uint8_t*)&Obs_buff_left[Obs_index_left], 1);
//     }

//     if (huart->Instance == USART3) {
//         if (++Obs_index_right >= 3) {
//             // 完成一帧数据接收
//             if ((Obs_buff_right[0] + Obs_buff_right[1]) == Obs_buff_right[2]) {
//                 Obs_distance_right = (Obs_buff_right[0] << 8) | Obs_buff_right[1];
//                 Obs_data_ready = 1;
//             } else {
//                 Obs_distance_right = 0xFFFF;
//             }
//             Obs_index_right = 0;
//         }

//         // 继续接收下一字节
//         HAL_UART_Receive_IT(&huart3, (uint8_t*)&Obs_buff_right[Obs_index_right], 1);
//     }
// }