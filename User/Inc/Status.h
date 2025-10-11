#ifndef __STATUS_H__
#define __STATUS_H__

typedef enum
{
	wait,
	left,
	right,
	forward,
	back_step,
	right_step,
	left_step,
	throw_rotate,
} Status_ID;
//up，down代表上层机构的投掷，up-投掷，down-复原

void Car_Status(Status_ID Status);
void Obstacle_find(void);
void Update_Status_Lib(void);
void Position_begin_predict(void);

#endif