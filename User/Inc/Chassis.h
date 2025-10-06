#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "main.h"

void Go(float Vx, float Vy, float Vw);
void Stop(uint32_t id);
void Stop_All();
void Start_Trace();
void Rotation_Deg(float Degree,uint32_t dir);
void Go_Right(uint32_t dis);
void Get_Back(uint32_t dis);
void Go_Forward(uint32_t dis);
void Go_Left(uint32_t dis);

#endif