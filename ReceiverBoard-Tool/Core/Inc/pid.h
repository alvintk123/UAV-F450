#ifndef _PID_H_
#define _PID_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stdint.h"

void PID_Equation(float Error, float P, float I, float D, float* PrevError, float* PrevITerm, float dt, float* PIDResult, char type);
void PID_Reset(float* PrevErrorRateRoll, float* PrevErrorRatePitch, float* PrevErrorRateYaw, float* PrevITermRateRoll, float* PrevITermRatePitch, 
								float* PrevITermRateYaw, float* PrevErrorAngleRoll, float* PrevErrorAnglePitch, float* PrevITermAngleRoll, float* PrevITermAnglePitch,
									float* PrevErrorVerticalVelocity, float* PrevItermVerticalVelocity);
								
#endif
