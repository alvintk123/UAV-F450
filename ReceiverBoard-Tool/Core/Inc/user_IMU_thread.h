
#ifndef _USER_IMU_THREAD_H_
#define _USER_IMU_THREAD_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include "ControlMotor.h"


void IMU_Initization(void);
void IMU_Calibration(verticalState *vertState);
void IMU_GetDataAndProcessing(verticalState	*vertState);

#endif /*_USER_IMU_THREAD_H_*/
