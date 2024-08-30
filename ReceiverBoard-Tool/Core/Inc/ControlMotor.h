#ifndef _CONTROLMOTOR_H_
#define _CONTROLMOTOR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include "ms5611.h"

typedef struct controlMotor
{
	float thrustControl;
	float verticalVelocityControl;
	float yawControl;
	float pitchControl;
	float rollControl;
} controlMotor;

typedef struct verticalState
{
	float altitude;
	float velocity;
	float velocityOffset;
	bool isCalibVert;
}verticalState;


void startMotor(TIM_HandleTypeDef* htim, float minThrottle, float maxThrottle, bool isCalib);
void initMotor(TIM_HandleTypeDef* htim);
void calibESC(TIM_HandleTypeDef* htim, float minThrottle, float maxThrottle);
void runMotor(TIM_HandleTypeDef* htim, float throttle[4]);
bool checkCalibSensorBeforeStartMotor(GY86_MPU6050_t *mpu, MS5611_t *ms5611, verticalState *vertState);
bool startPrintVerticalVel(GY86_MPU6050_t *mpu, MS5611_t *ms5611);
float makeStableInputJoystick(float newValue, float oldValue, float percentboundbound, char typeOfControl);
#endif
