
#include "ControlMotor.h"
#include "main.h"
controlMotor Motor;
verticalState vertState;
void startMotor(TIM_HandleTypeDef* htim, float minThrottle, float maxThrottle, bool isCalib)
{
	initMotor(htim);
	if (isCalib)
	{
		calibESC(htim, minThrottle, maxThrottle);
	}
}
void initMotor(TIM_HandleTypeDef* htim)
{
	// Init motor 
	while(HAL_TIM_Base_Start(htim)!=HAL_OK){};
	while(HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1)!=HAL_OK){};
	while(HAL_TIM_PWM_Start(htim,TIM_CHANNEL_2)!=HAL_OK){};
	while(HAL_TIM_PWM_Start(htim,TIM_CHANNEL_3)!=HAL_OK){};
	while(HAL_TIM_PWM_Start(htim,TIM_CHANNEL_4)!=HAL_OK){};
}

void calibESC(TIM_HandleTypeDef* htim, float minThrottle, float maxThrottle)
{
	// Set Max Pulse
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_1, maxThrottle);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_2, maxThrottle);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_3, maxThrottle);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_4, maxThrottle);
	osDelay(3000);
	// Set Min Pulse
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_1, minThrottle);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_2, minThrottle);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_3, minThrottle);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_4, minThrottle);
	osDelay(2000);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_1, 20);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_2, 20);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_3, 20);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_4, 20);
	osDelay(1000);
}
	

void runMotor(TIM_HandleTypeDef* htim, float throttle[4])
{
	// Right Front
	// Right Back
	// Left Back
	// Left Front

	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_1, throttle[0]);
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_2, throttle[1]); 
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_3, throttle[2]); 
	__HAL_TIM_SetCompare(htim,TIM_CHANNEL_4, throttle[3]);
	osDelay(20); // 20 ms = 50 Hz
}


bool checkCalibSensorBeforeStartMotor(GY86_MPU6050_t *mpu, MS5611_t *ms5611, verticalState *vertState)
{
	if(mpu->isCalibGyro && mpu->isCalibMag && ms5611->isCalibAlt && vertState->isCalibVert)
	{
		return true;
	}
	return false;
}

bool startPrintVerticalVel(GY86_MPU6050_t *mpu, MS5611_t *ms5611)
{
	if(mpu->isCalibGyro && mpu->isCalibMag && ms5611->isCalibAlt)
	{
		return true;
	}
	return false;
}

float makeStableInputJoystick(float newValue, float oldValue, float percentbound, char typeOfControl)
{
	float boundMax;
	float boundMin;
	if (typeOfControl == 'T')
	{
		boundMax  = 100.0f;
		boundMin = 0.0f;
	}
	else if (typeOfControl == 'Y')
	{
		boundMax  = 30.0f;
		boundMin = 0.0f;
	}
	if (typeOfControl == 'R' || typeOfControl == 'P')
	{
		boundMax  = 30.0f;
		boundMin = 30.0f;
	}
	// confirm change thrust Increase thrust when higher than old thrust
	if (newValue >= oldValue)
	{
		// counter vibration
		if (newValue - oldValue >= percentbound*(boundMax-boundMin)) //percentCounterVibration //
		{
			oldValue = newValue;
		}
	}
	else if (newValue < oldValue)
	{
		// counter vibration
		if (oldValue - newValue >= percentbound*(boundMax-boundMin)) //percentCounterVibration //
		{
			oldValue = newValue;
		}
	}
	return oldValue;
	// when release joystick or vibratio
}
