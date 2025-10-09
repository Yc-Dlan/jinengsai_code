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
extern detect_dir detect_flag;
extern uint32_t Stop_All_flag;
extern enum Trace_Dir Trace_flag;
extern uint32_t act_flag;

extern unsigned int Sensor_Right[];
extern unsigned int Sensor_Left[];
extern float Obs_distance_front = 0;
extern float Obs_distance_left = 0;
extern float Obs_distance_right = 0;

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

unsigned int Status_lib[6]={0,0,0,0,-1,0};
//使用数组来记录小车的几个状态量，[0][1]存放当前小车经过的格数，0-前进，1-左右；[2][3]存放当前小车前进方向的障碍物情况，2-前进，3-左右；[4]存放当前位移朝向;[5]存放起点位置，0-左，1-右

int cross_v = 0; //纵向格数
int cross_h = 0; //横向格数
int obstacle_v = 0; //纵向障碍物 0-无 1-有
int obstacle_h = 0; //横向障碍物 0-无 1-有
int move_toward = -1; //当前运动方向 -1-前进v 1-向左向右h
int pos_begin = 0; //先默认左区启动

void Car_Status(Status_ID Status)
{
    Start_Trace();
    Go_Right(Right_dis);
    Go_Left(Left_dis);
    Go_Forward(Forward_dis);
    Get_Back(Back_dis);
    wait_ms(ms);

    static Status_ID current_status = wait;
    static Status_ID next_status = wait;

    switch (current_status)
    {
    case wait:
        Stop_All_flag = 1;
        Forward_flag = 0;
        Back_flag = 0;
        Left_flag = 0;
        Right_flag = 0;
        Trace_flag = 0;
        cross_h = 0;
        cross_v = 0;
        obstacle_h = 0; 
        move_toward = 0; 
        pos_begin = 0;
        Position_begin_predict();
        switch (pos_begin)
        {
        case 0:
            if (Obs_distance_front >= Obs_distance_right){
                move_toward = 1;
                next_status = right;
            }
            else{
                move_toward = -1;
                next_status = forward;
            }
            break;
        
        case 1:
            if (Obs_distance_front >= Obs_distance_left){
                move_toward = 1;
                next_status = left;
            }
            else{
                move_toward = -1;
                next_status = forward;
            }
            break;
        }
        break;
    
    case left:
        if ((obstacle_h && obstacle_v)||(cross_h == 7 && cross_v == 4)){
            next_status = right_step;
        }
        else{
            while ((! obstacle_h) || (cross_h != 7))
            {
                Trace_flag = Left;
                detect_flag = left_dir;
                Detect_Line_L(1);
                cross_h++;
            }
            move_toward = * (-1);
            next_status = forward;
        }
        break;
    
    case right:
        if ((obstacle_h && obstacle_v)||(cross_h == 7 && cross_v == 4)){
            next_status = left_step;
        }
        else{
            while ((! obstacle_h) || (cross_h != 7))
            {
                Trace_flag = Right;
                detect_flag = right_dir;
                Detect_Line_R(1);
                cross_h++;
            }
            move_toward = * (-1);
            next_status = forward;
        }
        break;

    case forward:
        if ((obstacle_h && cross_h == 7)||(obstacle_v && cross_v == 4)){
            next_status = back_step;
        }
        while ((! obstacle_v) || (cross_v != 4))
        {
            Trace_flag = Forward;
            detect_flag = forward_dir;
            Detect_Line_F(1);
            cross_v++;
        }
        move_toward = * (-1);
        switch (pos_begin)
        {
        case 0:
            next_status = right;
            break;
        
        case 1:
            next_status = left;
            break;
        }
        break;
    
    case back_step:
        while (obstacle_h)
        {
            Back_flag = 1;
            Get_Back(20); //单位格子位移量
            cross_v--;
        }
        move_toward = * (-1);
        if(cross_h == 7){
            switch (pos_begin)
            {
            case 0:
                next_status = left_step;
                break;
        
            case 1:
                next_status = right_step;
                break;
            }
        }
        else{
            switch (pos_begin)
            {
            case 0:
                next_status = right;
                break;
        
            case 1:
                next_status = left;
                break;
            }
        }
        break;
    
    case right_step:
        while (obstacle_v)
        {
            Right_flag = 1;
            Go_Right(20); //单位格子位移量
            cross_h--;
        }
        move_toward = * (-1);
        if(cross_v == 4){
            next_status = back_step;
        }
        else{
            next_status = forward;
        }
        break;
    
    case left_step:
        while (obstacle_v)
        {
            Left_flag = 1;
            Go_Left(20); //单位格子位移量
            cross_h--;
        }
        move_toward = * (-1);
        if(cross_v == 4){
            next_status = back_step;
        }
        else{
            next_status = forward;
        }
        break;
    }

    if (!(cross_h == 7 || cross_v == 4)){
        return Car_Status(next_status);
    }
    else{
        return Car_Status(wait);
    }
}

// 更新状态库
void Update_Status_Lib(void)
{
    Status_lib[0] = cross_v;
    Status_lib[1] = cross_h;
    Status_lib[2] = obstacle_v;
    Status_lib[3] = obstacle_h;
    Status_lib[4] = move_toward;
}

void Position_begin_predict(void)
{
    int num_R = 0;
    int num_L = 0;
    for(int i =0; i < 8; i++)
    {
        num_L = num_L + Sensor_Left[i];
        num_R = num_R + Sensor_Right[i];
    }
    if (num_L >= num_R){
        pos_begin = 1;
    }
    else{
        pos_begin = 0;
    }
}

void Obstacle_find(void)
{
    int thresold_obs = 20; //为了方便后续更换阈值，在此修改即可，后续均使用别名
    if (Obs_distance_front <= thresold_obs){
        obstacle_v = 1;
    }
    else{
        obstacle_v = 0;
    }
    switch (pos_begin)
    {
    case 0:
        if (Obs_distance_right <= thresold_obs){
            obstacle_h = 1;
        }
        else{
            obstacle_h = 0;
        }
        break;
    case 1:
        if (Obs_distance_left <= thresold_obs){
            obstacle_h = 1;
        }
        else{
            obstacle_h = 0;
        }
        break;
    }
}
