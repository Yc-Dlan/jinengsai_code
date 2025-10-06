#ifndef __SENSOR_H__
#define __SENSOR_H__
#include "main.h"
#include "stdint.h"


#define GW_GRAY_ADDR_DEF1 0x4C
#define GW_GRAY_ADDR_DEF2 0x4C
#define GW_GRAY_PING 0xAA
#define GW_GRAY_PING_OK 0x66
#define GW_GRAY_PING_RSP GW_GRAY_PING_OK

#define Color_Adress 	0x48
#define RGB_Reg 			0xD0
#define HSL_Reg 			0xD1
#define PING 					0xAA
#define Error 				0xDE
#define Reset 				0xC0
#define Version				0xC1
#define PING_OK				0x66


#define GW_GRAY_DIGITAL_MODE 0xDD


#define GW_GRAY_ANALOG_BASE_ 0xB0
#define GW_GRAY_ANALOG_MODE  (GW_GRAY_ANALOG_BASE_ + 0)


#define GW_GRAY_ANALOG_NORMALIZE 0xCF


#define GW_GRAY_ANALOG(n) (GW_GRAY_ANALOG_BASE_ + (n))


#define GW_GRAY_CALIBRATION_BLACK 0xD0

#define GW_GRAY_CALIBRATION_WHITE 0xD1


#define GW_GRAY_ANALOG_CHANNEL_ENABLE 0xCE
#define GW_GRAY_ANALOG_CH_EN_1 (0x1 << 0)
#define GW_GRAY_ANALOG_CH_EN_2 (0x1 << 1)
#define GW_GRAY_ANALOG_CH_EN_3 (0x1 << 2)
#define GW_GRAY_ANALOG_CH_EN_4 (0x1 << 3)
#define GW_GRAY_ANALOG_CH_EN_5 (0x1 << 4)
#define GW_GRAY_ANALOG_CH_EN_6 (0x1 << 5)
#define GW_GRAY_ANALOG_CH_EN_7 (0x1 << 6)
#define GW_GRAY_ANALOG_CH_EN_8 (0x1 << 7)
#define GW_GRAY_ANALOG_CH_EN_ALL (0xFF)


#define GW_GRAY_ERROR 0xDE


#define GW_GRAY_REBOOT 0xC0


#define GW_GRAY_FIRMWARE 0xC1


#define GET_NTH_BIT(sensor_value, nth_bit) (((sensor_value) >> ((nth_bit)-1)) & 0x01)



#define SEP_ALL_BIT8(sensor_value, val1, val2, val3, val4, val5, val6, val7, val8) \
do {                                                                              \
val1 = GET_NTH_BIT(sensor_value, 1);                                              \
val2 = GET_NTH_BIT(sensor_value, 2);                                              \
val3 = GET_NTH_BIT(sensor_value, 3);                                              \
val4 = GET_NTH_BIT(sensor_value, 4);                                              \
val5 = GET_NTH_BIT(sensor_value, 5);                                              \
val6 = GET_NTH_BIT(sensor_value, 6);                                              \
val7 = GET_NTH_BIT(sensor_value, 7);                                              \
val8 = GET_NTH_BIT(sensor_value, 8);                                              \
} while(0)

#define GW_GRAY_CHANGE_ADDR 0xAD

#define GW_GRAY_BROADCAST_RESET "\xB8\xD0\xCE\xAA\xBF\xC6\xBC\xBC"

#define Offset 0x88
#if defined (ESP_PLATFORM)
/* ESP32 */


#endif


void Get_Sensor(unsigned int *Sensor_Front);
float Speed_Correction(void);
int Get_Color(void);
int Exposure(uint32_t color);
int Detect_Line_F(uint32_t count);
int Detect_Line_L(uint32_t count);
int Detect_Line_R(uint32_t count);

#endif