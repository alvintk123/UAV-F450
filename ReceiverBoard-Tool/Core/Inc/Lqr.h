#ifndef _LQR_H_
#define _LQR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stdint.h"
#include "ms5611.h"
#include "ControlMotor.h"
#include "equation.h"

void LQRController(GY86_MPU6050_t *mpu, MS5611_t *MS5611, verticalState	*vertState, float preIntegErrState[4][1], float prevErrState[4], float errState[4], float U[4][1], float dt);

#endif
