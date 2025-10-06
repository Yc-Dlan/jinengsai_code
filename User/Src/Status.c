#include "main.h"
#include "StepperMotor.h"
#include "tim.h"
#include "stdlib.h"
#include "Chassis.h"
#include "Math.h"
#include "stdio.h"
#include "Status.h"
#include "Servo.h"

Status_ID Status = wait;

extern float ms;
extern uint32_t wait_finish;
extern uint32_t wait_flag;
extern float wait_sec;
extern uint32_t detect_flag;
extern uint32_t Stop_All_flag;
extern enum Trace_Dir Trace_flag;
extern uint32_t act_flag;

extern uint32_t Rotation_flag;
extern uint32_t Back_flag;
extern uint32_t Back_count;
extern uint32_t Forward_flag;
extern uint32_t Forward_count;
extern uint32_t Rotation_count;
extern uint32_t Left_flag;
extern uint32_t Left_count;
extern uint32_t Right_flag;
extern uint32_t Right_count;
extern uint32_t Right_dis;
extern uint32_t Left_dis;
extern uint32_t Back_dis;
extern uint32_t Forward_dis;
extern float Rotation_Degree;

void Car_Status(Status_ID Status)
{
	Start_Trace();
    Go_Right(Right_dis);
    Go_Left(Left_dis);
    Go_Forward(Forward_dis);
    Get_Back(Back_dis);
    wait_ms(ms);
    if (Detect_Line_L(8))
    {
        detect_flag = 1;
        Trace_flag = NO;
        act_flag = 1;
        Stop_All_flag = 1;
    }
    if (act_flag == 1)
    {
        if (wait_fun(1))
        {
            act_flag = 2;
        }
    }
    if (act_flag == 2)
    {
        Trace_flag = Forward;
        if (Detect_Line_F(4))
        {
            Stop_All_flag = 1;
            Trace_flag = NO;
            act_flag = 3;
        }
    }
    wait_ms(ms);
}