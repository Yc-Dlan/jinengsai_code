#include "main.h"
#include "StepperMotor.h"
#include "tim.h"
#include "stdlib.h"
#include "Chassis.h"
#include "Math.h"
#include "stdio.h"
#include "Status.h"
#include "Servo.h"
#include "Ultrasonic.h"

Status_ID Status = wait;

extern float ms;
extern uint32_t wait_finish;
extern uint32_t wait_flag;
extern float wait_sec;
extern detect_dir detect_flag;
extern uint32_t Stop_All_flag;
extern enum Trace_Dir Trace_flag;
extern Move_Status Move_status;
extern uint32_t act_flag;

extern unsigned int Sensor_Right[];
extern unsigned int Sensor_Left[];

extern uint32_t Back_count;
extern uint32_t Forward_count;
extern uint32_t Rotation_count;
extern uint32_t Left_count;
extern uint32_t Right_count;
extern uint32_t Right_dis;
extern uint32_t Left_dis;
extern uint32_t Back_dis;
extern uint32_t Forward_dis;
extern float Rotation_Degree;
extern uint32_t arm_flag;
uint32_t next_step = 0;
uint32_t cycle = 0;

float angle = 0;

unsigned int Status_lib[6] = {0, 0, 0, 0, -1, 0};
// 使用数组来记录小车的几个状态量，[0][1]存放当前小车经过的格数，0-前进，1-左右；[2][3]存放当前小车前进方向的障碍物情况，2-前进，3-左右；[4]存放当前位移朝向;[5]存放起点位置，0-左，1-右

int cross_v = 0;      // 纵向格数
int cross_h = 0;      // 横向格数
int obstacle_v = 0;   // 纵向障碍物 0-无 1-有
int obstacle_h = 0;   // 横向障碍物 0-无 1-有
int move_toward = -1; // 当前运动方向 -1-前进v 1-向左向右h
int pos_begin = 1;    // 先默认左区启动

void Car_Status(Status_ID status)
{
    Start_Trace();
    Go_Right(Right_dis);
    Go_Left(Left_dis);
    Go_Forward(Forward_dis);
    Get_Back(Back_dis);
    Set_Angle(angle);
    Rotation_Deg(Rotation_Degree, 1);

    switch (Status)
    {
    case wait:
        Stop_All_flag = 1;
        Move_status = no;
        Trace_flag = 0;
        arm_flag = 0;
        cross_h = 0;
        cross_v = 0;
        obstacle_h = 0;
        move_toward = 0;
        pos_begin = 0;
        Position_begin_predict();
        switch (pos_begin)
        {
        case 0:
            if (Obs_distance_front >= Obs_distance_right)
            {
                move_toward = 1;
                Status = forward;
            }
            else
            {
                move_toward = -1;
                Status = right;
            }
            break;

        case 1:
            if (Obs_distance_front >= Obs_distance_left)
            {
                move_toward = 1;
                Status = forward;
            }
            else
            {
                move_toward = -1;
                Status = left;
            }
            break;
        }
        Update_Status_Lib();
        Status_lib[5] = pos_begin;
        break;

    case left:
        if ((obstacle_h || cross_h == 7) && (obstacle_v || cross_v == 4))
        {
            Status = right_step;
        }
        if ((!obstacle_h) && (cross_h != 7))
        {
            Trace_flag = Left;
            detect_flag = left_dir;
            if (Detect_Line_L(1))
            {
                cross_h++;
            }
        }
        else
        {
            Stop_All_flag = 1;
            if (wait_fun(1))
            {
                move_toward = -move_toward;
                Status = forward;
            }
        }
        Update_Status_Lib();
        break;

    case right:
        if ((obstacle_h || cross_h == 7) && (obstacle_v || cross_v == 4))
        {
            Status = left_step;
        }
        if ((!obstacle_h) && (cross_h != 7))
        {
            Trace_flag = Right;
            detect_flag = right_dir;
            if (Detect_Line_R(1))
            {
                cross_h++;
            }
        }
        else
        {
            if (wait_fun(0.05))
            {
                Stop_All_flag = 1;
                move_toward = -move_toward;
                Status = forward;
            }
        }
        Update_Status_Lib();
        break;

    case forward:
        if ((obstacle_h || cross_h == 7) && (obstacle_v || cross_v == 4))
        {
            Status = back_step;
        }
        if ((!obstacle_v) && (cross_v != 4))
        {
            Trace_flag = Forward;
            detect_flag = forward_dir;
            if (Detect_Line_F(1))
            {
                cross_v++;
            }
        }
        else
        {
            if (wait_fun(0.35))
            {
                Stop_All_flag = 1;
                switch (pos_begin)
                {
                case 0:
                    Status = right;
                    break;

                case 1:
                    Status = left;
                    break;
                }
                move_toward = -move_toward;
            }
        }
        Update_Status_Lib();
        break;

    case back_step:
        if (obstacle_h)
        {
            if (wait_fun(1))
            {
                Move_status = Back_flag;
                Back_dis = 180; // 单位格子位移量
                cross_v--;
            }
        }
        else
        {
            if (cross_h == 7)
            {
                switch (pos_begin)
                {
                case 0:
                    Status = left_step;
                    break;

                case 1:
                    Status = right_step;
                    break;
                }
            }
            else
            {
                switch (pos_begin)
                {
                case 0:
                    Status = right;
                    break;

                case 1:
                    Status = left;
                    break;
                }
            }
        }
        Update_Status_Lib();
        break;

    case right_step:
        if (obstacle_v)
        {
            if (wait_fun(1))
            {
                Trace_flag = Right;
                detect_flag = right_dir;
                if (Detect_Line_R(1))
                {
                    cross_h--;
                }
            }
        }
        else
        {
            move_toward = -move_toward;
            if (cross_v == 4)
            {
                Status = back_step;
            }
            else
            {
                Status = forward;
            }
        }
        Update_Status_Lib();
        break;

    case left_step:
        if (obstacle_v)
        {
            if (wait_fun(1))
            {
                Trace_flag = Left;
                detect_flag = left_dir;
                if (Detect_Line_L(1))
                {
                    cross_h--;
                }
            }
        }
        else
        {
            move_toward = -move_toward;
            if (cross_v == 4)
            {
                Status = back_step;
            }
            else
            {
                Status = forward;
            }
        }
        Update_Status_Lib();
        break;

    case throw_rotate:
        switch (next_step)
        {
        case 0:
            if (wait_fun(1))
            {
                Move_status = Forward_flag;
                Forward_dis = 20;
                next_step = 1;
            }
            break;
        case 1:
            if (Move_status == no)
            {
                arm_flag = 1;
                angle = Pul_Angel;
                next_step = 2;
            }
            break;
        case 2:
            if (arm_flag == 0)
            {
                if (wait_fun(1))
                {
                    arm_flag = 1;
                    angle = -Pul_Angel;
                    next_step = 3;
                }
            }
            break;
        case 3:
            if (arm_flag == 0)
            {
                Move_status = Back_flag;
                Back_dis = 80;
                next_step = 4;
            }
            break;
        case 4:
            if (Move_status == no)
            {
                if (wait_fun(1))
                {
                    Move_status = Rotation_flag;
                    Rotation_Degree = 50;
                    next_step = 5;
                }
            }
            break;
        case 5:
            if (Move_status == no)
            {
                if (wait_fun(1))
                {
                    Status = wait;
                    cross_h = 0;
                    cross_v = 0;
                    next_step = 0;
                }
            }
            break;
        default:
            break;
        }
        break;

    case finish:
        if (wait_fun(3))
        {
            Move_status = Rotation_flag;
            Rotation_Degree = 180;
            next_step = 1;
        }
        if (next_step == 1 && Move_status == no)
        {
            Status = wait;
            cross_h = 0;
            cross_v = 0;
        }
    }

    if (!(cross_h == 7 && cross_v == 4))
    {
    }
    else
    {
        cycle++;
        if (cycle % 2 == 1)
        {
            Status = throw_rotate;
        }
        else
        {
            Status = finish;
        }
    }
    wait_ms(ms);
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
    for (int i = 0; i < 8; i++)
    {
        num_L = num_L + Sensor_Left[i];
        num_R = num_R + Sensor_Right[i];
    }
    if (num_L >= num_R)
    {
        pos_begin = 1;
    }
    else
    {
        pos_begin = 0;
    }
}

void Obstacle_find(void)
{
    int thresold_obs = 250; // 为了方便后续更换阈值，在此修改即可，后续均使用别名
    if ((Obs_distance_front <= thresold_obs) || cross_v == 4)
    {
        obstacle_v = 1;
    }
    else
    {
        obstacle_v = 0;
    }
    switch (pos_begin)
    {
    case 0:
        if ((Obs_distance_right <= thresold_obs) || cross_h == 7)
        {
            obstacle_h = 1;
        }
        else
        {
            obstacle_h = 0;
        }
        break;
    case 1:
        if ((Obs_distance_left <= thresold_obs) || cross_h == 7)
        {
            obstacle_h = 1;
        }
        else
        {
            obstacle_h = 0;
        }
        break;
    }
}
