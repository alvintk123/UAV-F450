
#ifndef _USER_MAIN_THREAD_H_
#define _USER_MAIN_THREAD_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stdint.h"
#include "cmsis_os2.h"
#include "ControlMotor.h"
#include "FlightController.h"
#include "stdbool.h"

#define MIN_THROTTLE 550
#define MAX_THROTTLE 970


// declare extern variable
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;

// declare function
void Motor_Initization(void);
void Motor_GetDataAndProcessing(verticalState	*vertState);

//void Main_EnableAllPWM_Chanel(void);
//void Main_TestALLMotor(void);
//void Main_ChangeAllMotor(void);



#endif /*_USER_MAIN_THREAD_H_*/
