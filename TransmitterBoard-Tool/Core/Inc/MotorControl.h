
#include "stdbool.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

#ifndef _MOTORCONTROL_H_
#define _MOTORCONTROL_H_
// Kalman filter

typedef enum SendCommand {
	No,
	Yes,
}SendCommand;

typedef struct controlMotor
{
	float thrustControl;
	float yawControl;
	float pitchControl;
	float rollControl;
} controlMotor;


#endif
