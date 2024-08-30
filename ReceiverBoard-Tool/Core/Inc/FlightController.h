#ifndef _FLIGHTCONTROLLER_H_
#define _FLIGHTCONTROLLER_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include "stdint.h"
#include "ms5611.h"
#include "pid.h"
#include "ControlMotor.h"
#include "math.h"
#include "equation.h"

void ControlLoop(controlMotor* control, verticalState* vertState, float dt, bool checkStartMotor);

#endif

