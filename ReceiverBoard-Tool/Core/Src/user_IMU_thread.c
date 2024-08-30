
#include "user_IMU_thread.h"
#include "main.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "hmc5883l.h"
#include "ControlMotor.h"
#include "flash.h"

extern MS5611_t 								ms5611_1;
extern MS5611_t 								ms5611_2;
extern GY86_MPU6050_t 					MPU6050;
extern I2C_HandleTypeDef 				hi2c1;
extern I2C_HandleTypeDef 				hi2c3;
extern bool 										isStartMotor;
extern APPLICATION_ADC_INFO			app_adc_infor;
extern APPLICATION_SLAVE_INFO		app_slave_infor;

uint32_t tickCount_IMU;
uint32_t loopTimer1_IMU;
uint32_t loopTimer2_IMU;
uint32_t tickFreq;
float		 deltaT_IMU;
int 		 stableTime 					 = 0;
bool 		 isCalibVerticalVel 	 = false;
float 	 valueCalibVerticalVel = 0;

float 	 vertStateTemp[2][1] 	 = {{0},{0}};
float 	 P[2][2] 							 = {{0,0},{0,0}};
int  		 count = 0;
int  		 count3 = 0;

bool 		 isConfirmSaveHMC5883  = false;
bool 		 isConfirmLoadHMC5883  = false;

/*
Kalman filter variable for vertical state
vertStateTemp = {Altitude, Vertical Velocity}
*/

void IMU_Initization(void)
{

	/*
		Init and calib MPU6050
		-> Turn on Led 11 when finish calib MPU6050
		-> Turn on Led 11 two times when failing init MPU6050
	*/
	
	MPU6050_Start(&hi2c1, &MPU6050, 1, 1, 5);
	
	/*
		Init and Calib HMC5883L
		-> Turn on Led 12 when finish calib HMC5883L
	*/
	
	HMC5883L_Start(&hi2c1, &MPU6050);
	
	/*
		Init and Calib MS5611
		-> Turn off Led 11 and 12 when finish calib HMC5883L
	*/
	
	MS5611_Start(&hi2c1, &ms5611_1, &count);
	
//	MS5611_Start(&hi2c3, &ms5611_2, &count3);
	tickFreq 			= osKernelGetTickFreq();
	tickCount_IMU = 20U * tickFreq/ 1000u; // for 20 ms = 50 HZ
	deltaT_IMU 		= 6;
}

void IMU_Calibration(verticalState	*vertState)
{
	app_slave_infor.mpu6050CalibFlag = 1;  // is calibrating
	CalibMPU6050(&hi2c1, &MPU6050);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
	osDelay(1000);
	app_slave_infor.mpu6050CalibFlag = 2;  // calib successful
	
	app_slave_infor.hmc5883lCalibFlag = 1;
	CalibHMC5883L(&hi2c1, &MPU6050);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	osDelay(1000);
	app_slave_infor.hmc5883lCalibFlag = 2;
	
	app_slave_infor.ms5611CalibFlag = 1; 
	MS5611_CalibAltitude(&hi2c1, &ms5611_1, &count);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
	osDelay(1000);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	osDelay(1000);
	app_slave_infor.ms5611CalibFlag = 2;
	
	while(stableTime <= 2001) // 2501
	{
		app_slave_infor.vertVelCalibFlag = 1;
		loopTimer1_IMU 				= osKernelGetTickCount();
		if (stableTime > 1000 && isCalibVerticalVel)
		{
			if (stableTime <= 2000) // 2500 
			{
				vertState->isCalibVert = false;
				stableTime += 1;
				valueCalibVerticalVel += vertStateTemp[1][0];
			}
			else
			{
				valueCalibVerticalVel /= 1000.0f; // 1500
				vertState->velocityOffset = valueCalibVerticalVel;
				vertState->isCalibVert = true;
				isCalibVerticalVel = false;
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
				osDelay(1000);
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
				osDelay(1000);
				stableTime 				 += 1;
			}
		
		}
		else if (stableTime < 1000 && !isCalibVerticalVel)
		{
			stableTime 					 += 1;
		}
		else if (stableTime == 1000)
		{
			isCalibVerticalVel 	 = true;
			stableTime 				   += 1;
		}
				
		GetRollPitchYawAngle(&hi2c1, &MPU6050, deltaT_IMU/1000.0f);
		MS5611_GetAltitudeVerticalVelocity(&hi2c1, &hi2c1, &MPU6050, &ms5611_1, vertStateTemp, P, &count, 1, deltaT_IMU/1000.0f);
		loopTimer2_IMU 				 		= osKernelGetTickCount();
		
		deltaT_IMU 								= (loopTimer2_IMU - loopTimer1_IMU)*1000u/tickFreq;
//		app_slave_infor.deltaTImu = deltaT_IMU;
		
		while (loopTimer2_IMU - loopTimer1_IMU < tickCount_IMU)
		{
			loopTimer2_IMU 			 		= osKernelGetTickCount();
		}
	}
	
	app_slave_infor.vertVelCalibFlag = 2;
}
void IMU_GetDataAndProcessing(verticalState	*vertState)
{
	/*
	Calib value for vertical velocity
	*/
	
//	while(stableTime <= 2501)
//	{
//		app_slave_infor.vertVelCalibFlag = 1;
//		loopTimer1_IMU 				= osKernelGetTickCount();
//		if (stableTime > 1000 && isCalibVerticalVel)
//		{
//			if (stableTime <= 2500)
//			{
//				vertState->isCalibVert = false;
//				stableTime += 1;
//				valueCalibVerticalVel += vertStateTemp[1][0];
//			}
//			else
//			{
//				valueCalibVerticalVel /= 1500.0f;
//				vertState->velocityOffset = valueCalibVerticalVel;
//				vertState->isCalibVert = true;
//				isCalibVerticalVel = false;
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
//				osDelay(1000);
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
//				osDelay(1000);
//				stableTime 				 += 1;
//			}
//		
//		}
//		else if (stableTime < 1000 && !isCalibVerticalVel)
//		{
//			stableTime 					 += 1;
//		}
//		else if (stableTime == 1000)
//		{
//			isCalibVerticalVel 	 = true;
//			stableTime 				   += 1;
//		}
//				
//		GetRollPitchYawAngle(&hi2c1, &MPU6050, deltaT_IMU/1000.0f);
//		MS5611_GetAltitudeVerticalVelocity(&hi2c1, &hi2c1, &MPU6050, &ms5611_1, vertStateTemp, P, &count, 1, deltaT_IMU/1000.0f);
//		loopTimer2_IMU 				 		= osKernelGetTickCount();
//		
//		deltaT_IMU 								= (loopTimer2_IMU - loopTimer1_IMU)*1000u/tickFreq;
//		app_slave_infor.deltaTImu = deltaT_IMU;
//		
//		while (loopTimer2_IMU - loopTimer1_IMU < tickCount_IMU)
//		{
//			loopTimer2_IMU 			 		= osKernelGetTickCount();
//		}
//	}
//	
//	app_slave_infor.vertVelCalibFlag = 2;
	
	// ------------------------------ Save and Load callib data -------------------------------------- 
	/*
	if (app_adc_infor.isSaveCalibData)
	{
		isConfirmSaveHMC5883 	 		= false;
	}
	if (app_adc_infor.isSaveHMC5883)
	{
		int16_t dataHMCToSave[6] 	= { MPU6050.Mag_X_Max, MPU6050.Mag_X_Min, MPU6050.Mag_Y_Max, MPU6050.Mag_Y_Min, MPU6050.Mag_Z_Max, MPU6050.Mag_Z_Min};
		saveDataToFlash(dataHMCToSave, sizeof(dataHMCToSave)/sizeof(dataHMCToSave[0]));
		isConfirmSaveHMC5883 	 		= true;
	}
	if (app_adc_infor.isLoadCalibData)
	{
		isConfirmLoadHMC5883 	 		= false;
	}
	if (app_adc_infor.isLoadHMC5883)
	{
		int16_t dataHMCToLoad[6];
		loadDataFromFlash(dataHMCToLoad, sizeof(dataHMCToLoad)/sizeof(dataHMCToLoad[0]));
		MPU6050.Mag_X_Max 		 = dataHMCToLoad[0];
		MPU6050.Mag_X_Min 		 = dataHMCToLoad[1];
		MPU6050.Mag_Y_Max 		 = dataHMCToLoad[2];
		MPU6050.Mag_Y_Min 		 = dataHMCToLoad[3];
		MPU6050.Mag_Z_Max 		 = dataHMCToLoad[4];
		MPU6050.Mag_Z_Min 		 = dataHMCToLoad[5];
		MPU6050.Mag_X_Offset 	 = (MPU6050.Mag_X_Max + MPU6050.Mag_X_Min) / 2;
		MPU6050.Mag_Y_Offset 	 = (MPU6050.Mag_Y_Max + MPU6050.Mag_Y_Min) / 2;
		MPU6050.Mag_Z_Offset 	 = (MPU6050.Mag_Z_Max + MPU6050.Mag_Z_Min) / 2;
		isConfirmLoadHMC5883 	 = true;
	}
	*/
	//----------------------------------------------------------------------------------------------------------
	
	
	loopTimer1_IMU  				 = osKernelGetTickCount();
	GetRollPitchYawAngle(&hi2c1, &MPU6050, deltaT_IMU/1000.0f);
	MS5611_GetAltitudeVerticalVelocity(&hi2c1, &hi2c1, &MPU6050, &ms5611_1, vertStateTemp, P, &count, 1, deltaT_IMU/1000.0f);
	vertState->altitude 		 = vertStateTemp[0][0];
	vertState->velocity 		 = vertStateTemp[1][0] - vertState->velocityOffset;
	
//	if (!isStartMotor)
//	{
//		if (vertState->velocity != 0)
//		{
//			vertState->velocityOffset += vertState->velocity;
//		}
//	}
	
	// Wait for 10ms for control motor
	loopTimer2_IMU 						= osKernelGetTickCount();
	while (loopTimer2_IMU - loopTimer1_IMU <= tickCount_IMU)
	{
		loopTimer2_IMU 				  = osKernelGetTickCount();
	}
	
	deltaT_IMU 								= (loopTimer2_IMU - loopTimer1_IMU)*1000u/tickFreq;
//	app_slave_infor.deltaTImu = deltaT_IMU;
}

