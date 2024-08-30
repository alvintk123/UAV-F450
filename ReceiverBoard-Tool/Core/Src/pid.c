
#include "pid.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "math.h"

void PID_Equation(float Error, float P, float I, float D, float* PrevError, float* PrevITerm, float dt, float* PIDResult, char type)
{
	float maxPIDOutput;
	float minPIDOutput;
	
	if (type == 'P')
	{
		maxPIDOutput = 45*3.14/180;
		minPIDOutput = -45*3.14/180;;
	}
	if (type == 'R')
	{
		maxPIDOutput = 45*3.14/180;;
		minPIDOutput = -45*3.14/180;;
	}
	if (type == 'T')
	{
		maxPIDOutput = 57.6;
		minPIDOutput = -57.6;
	}
	else if (type == 'L')
	{
		maxPIDOutput = 4.3571;
		minPIDOutput = -4.3571;
	}
	else if (type == 'M')
	{
		maxPIDOutput = 4.3299;
		minPIDOutput = -4.3299;
	}
	else if (type == 'N')
	{
		maxPIDOutput = 0.25;
		minPIDOutput = -0.25;
	}
	
	float PTerm = P * Error;
	float ITerm = *PrevITerm + I*(Error+*PrevError)*dt/2;
	if (ITerm > maxPIDOutput) ITerm = maxPIDOutput;
	else if (ITerm < minPIDOutput) ITerm = minPIDOutput;
	float DTerm = D*(Error-*PrevError)/dt;
	float PIDOutput = PTerm + ITerm + DTerm;
	if (PIDOutput > maxPIDOutput) PIDOutput = maxPIDOutput;
	else if (PIDOutput < minPIDOutput) PIDOutput = minPIDOutput;
	
	*PIDResult = PIDOutput;
	*PrevITerm = ITerm;
	*PrevError = Error;
	
//	float PTerm = P * Error;
//	float ITerm = *PrevITerm + I*(Error+*PrevError)*(1/Freq)/2;
//	if (ITerm > 80) ITerm = 80;
//	else if (ITerm < -20) ITerm = -20;
//	float DTerm = D*(Error-*PrevError)/(1/Freq);
//	float PIDOutput = PTerm + ITerm + DTerm;
//	if (PIDOutput > 80) PIDOutput = 80;
//	else if (PIDOutput < -20) PIDOutput = -20;
//	
//	*PIDResult = PIDOutput;
//	*PrevITerm = ITerm;
//	*PrevError = Error;
	
}

void PID_Reset(float* PrevErrorRateRoll, float* PrevErrorRatePitch, float* PrevErrorRateYaw, float* PrevITermRateRoll, float* PrevITermRatePitch, 
								float* PrevITermRateYaw, float* PrevErrorAngleRoll, float* PrevErrorAnglePitch, float* PrevITermAngleRoll, float* PrevITermAnglePitch,
									float* PrevErrorVerticalVelocity, float* PrevItermVerticalVelocity)
{
	*PrevErrorRateRoll = 0;
	*PrevErrorRatePitch = 0;
	*PrevErrorRateYaw = 0;
	*PrevITermRateRoll = 0;
	*PrevITermRatePitch = 0;
	*PrevITermRateYaw = 0;
	*PrevErrorAngleRoll = 0;
	*PrevErrorAnglePitch = 0;
	*PrevITermAngleRoll = 0;
	*PrevITermAnglePitch = 0;
	*PrevErrorVerticalVelocity = 0;
	*PrevItermVerticalVelocity = 0;
}
