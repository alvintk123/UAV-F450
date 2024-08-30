
#include "user_main_thread.h"
#include "main.h"
#include "equation.h"

extern MS5611_t 								ms5611_1;
extern GY86_MPU6050_t 					MPU6050; 
extern controlMotor 					  Motor;
extern uint32_t 								tickFreq;
extern APPLICATION_ADC_INFO			app_adc_infor;
extern APPLICATION_SLAVE_INFO		app_slave_infor;

// Loop timer for control motor 
uint32_t loopTimer1_FL;
uint32_t loopTimer2_FL;

uint32_t tickCount_20ms;
uint32_t tickCount_1ms;
uint32_t tickCount_MT;
float		 deltaT_FL							 = 10;
bool 		 isStartMotor 				 	 = false;
bool 		 getFirstValueForControl = false;
bool 		 holdMotor 							 = false;
void Motor_Initization(void)
{
		startMotor(&htim4, MIN_THROTTLE, MAX_THROTTLE, true);
}

void Motor_GetDataAndProcessing(verticalState	*vertState)
{

//	if (checkCalibSensorBeforeStartMotor(&MPU6050, &ms5611_1, vertState))
//	{
		// Start count tick
		loopTimer1_FL = osKernelGetTickCount();
		tickCount_20ms = 20U * osKernelGetTickFreq() / 1000u; // for 10ms =100HZ
	
		Motor.thrustControl 					= app_adc_infor.thrustControl;
		Motor.yawControl 							= app_adc_infor.yawControl;
		Motor.pitchControl 						= app_adc_infor.pitchControl;
		Motor.rollControl 						= app_adc_infor.rollControl;
	/*
		if (!getFirstValueForControl)
		{
			Motor.thrustControl 					= app_adc_infor.thrustControl;
			Motor.yawControl 							= app_adc_infor.yawControl;
			Motor.pitchControl 						= app_adc_infor.pitchControl;
			Motor.rollControl 						= app_adc_infor.rollControl;
			Motor.verticalVelocityControl = map(Motor.thrustControl, 0, 100, -1, 1);
			getFirstValueForControl 			= true;
		}
		else
		{
			Motor.thrustControl 					= makeStableInputJoystick(app_adc_infor.thrustControl, Motor.thrustControl, 0.05f, 'T');
			Motor.yawControl 							= makeStableInputJoystick(app_adc_infor.yawControl, Motor.yawControl,  0.05f, 'Y');
			if (Motor.yawControl > -5.0f && Motor.yawControl < 5.0f)
			{
				Motor.yawControl = 0;
			}
			Motor.pitchControl 						= makeStableInputJoystick(app_adc_infor.pitchControl, Motor.pitchControl,  0.05f, 'P');
			if (Motor.pitchControl > -5.0f && Motor.pitchControl < 5.0f)
			{
				Motor.pitchControl = 0.0f;
			}
			Motor.rollControl 						= makeStableInputJoystick(app_adc_infor.rollControl, Motor.rollControl,  0.05f, 'R');
			if (Motor.rollControl > -5.0f && Motor.rollControl < 5.0f)
			{
				Motor.rollControl = 0.0f;
			}
			Motor.verticalVelocityControl = map(Motor.thrustControl, 0.0f, 100.0f, -1.5f, 1.5f);
		}
	*/
		
		// Tune PID
		if (app_adc_infor.isTunePID && app_adc_infor.isTuneThrottle)
		{
			isStartMotor = false; // bug: start motor is not only depend on isStartMotor variable but also depend on app_adc_infor.isOnMotor variable
		}
		else
		{
			/*
				Stop motor & Start motor:
				Joystick left -> pull right down
				Joystick right ->  pull left down
				*/
			// Turn on motor
			if (((Motor.thrustControl<20 && Motor.yawControl > 25 && Motor.rollControl < -25 && Motor.pitchControl < -25) || app_adc_infor.isOnMotor) && !isStartMotor) 
			{
					isStartMotor = !isStartMotor;
					holdMotor 	 = true;
			}
			// Turn off motor
			if (((Motor.thrustControl>80 && Motor.yawControl > 25 && Motor.rollControl < -25 && Motor.pitchControl > 25) || (!app_adc_infor.isOnMotor)) && isStartMotor) 
			{
					isStartMotor = !isStartMotor; 
			}
		}
		
		if (isStartMotor)
		{
			if (holdMotor)
			{
				float initMotor[4] = {650,650,650,650};
				runMotor(&htim4, initMotor);
				holdMotor 	 = false;
				osDelay(5000);
			}
			tickCount_MT = tickCount_20ms;
		}
		else
		{
			tickCount_MT = tickCount_20ms;
		}
		ControlLoop(&Motor, vertState, deltaT_FL/1000.0f, isStartMotor);
			// Wait for 10ms for control motor
		loopTimer2_FL							 = osKernelGetTickCount();
		deltaT_FL 								 = (loopTimer2_FL - loopTimer1_FL)*1000u/tickFreq;
//		app_slave_infor.deltaTMain = deltaT_FL;
		while (loopTimer2_FL - loopTimer1_FL < tickCount_MT)
		{
			loopTimer2_FL = osKernelGetTickCount();
		}
//	}
}
