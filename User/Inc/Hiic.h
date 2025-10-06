#ifndef __HIIC_H__
#define __HIIC_H__


#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "Sensor.h"

unsigned char Ping(void);
unsigned char Ping2(void);
unsigned char Ping3(void);
unsigned char IIC_Get_Digtal(void);
unsigned char IIC_Get_Digtal2(void);
unsigned char IIC_Get_Digtal3(void);
unsigned char IIC_Get_Anolog(unsigned char * Result,unsigned char len);
unsigned char IIC_Get_Single_Anolog(unsigned char Channel);
unsigned char IIC_Anolog_Normalize(uint8_t Normalize_channel);
unsigned short IIC_Get_Offset(void );

unsigned char IIC_Get_HSL(unsigned char * Result,unsigned char len);
unsigned char IIC_Get_RGB(unsigned char * Result,unsigned char len);


#endif