#include "main.h"
#include "stdint.h"

extern volatile uint32_t Obs_distance;    // 存储测量距离（单位：毫米）
extern volatile uint32_t Obs_buff_front;
extern volatile uint32_t Obs_buff_left;
extern volatile uint32_t Obs_buff_right;              // 接收数据缓冲区
extern volatile uint32_t Obs_data_ready; 

extern volatile float Obs_distance_front;
extern volatile float Obs_distance_left;
extern volatile float Obs_distance_right;

void Ultrasonic_Trig_front(void);
void Ultrasonic_Trig_left(void);
void Ultrasonic_Trig_right(void);
void Ultrasonic_Init(void);

