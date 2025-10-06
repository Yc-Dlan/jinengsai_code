#ifndef __STATUS_H__
#define __STATUS_H__

typedef enum
{
	wait,
	left,
	right,
	down,
	up,
} Status_ID;

void Car_Status(Status_ID Status);

#endif