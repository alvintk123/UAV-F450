#include "user_Main_thread.h"
// Data for Sending
extern uint8_t 			isConnectVehicleFlag;
extern uint8_t   		isCalibSensorFlag;
extern uint8_t   		isStartSensorFlag;
extern uint8_t   		isCalibMotorFlag;
extern uint8_t   		isOnMotorFlag;
extern uint8_t   		isStateResponseFlag;
extern uint8_t   		isPlot3DAnimationFlag;
extern uint8_t   		isPlotRefResFlag;

// Data Receiving from controller tool
extern uint8_t				getConnectVehicle;
extern uint8_t				startCalibSensor;
extern uint8_t				startSensor;
extern uint8_t				startCalibMotor;
extern uint8_t				startMotor;
extern uint8_t				getStateResponse;
extern uint8_t				plot3DAnimation;
extern uint8_t				plotRefRes;

extern APPLICATION_ADC_INFO					app_adc_infor;
extern APPLICATION_SLAVE_INFO				app_slave_infor;

extern bool 												isSendFeedbackFlag;
extern bool													isSendFeedbackFlagState;

void Main_Processing(void)
{
	sensorFeedback(&app_slave_infor, &isCalibSensorFlag);
	
	// Start connect vehicle
	if (getConnectVehicle)
	{	
		if ((HAL_GPIO_ReadPin(Xbee_Master_GPIO_Port, Xbee_Master_Pin) != GPIO_PIN_SET))
		{
			HAL_GPIO_WritePin(Xbee_Master_GPIO_Port, Xbee_Master_Pin, GPIO_PIN_SET);
		}
		if (app_slave_infor.isConnectFlag)
		{
			isConnectVehicleFlag = 1;
		}
	}
	/*
		Processing Sensor
	*/
	// Start calib sensor
	if (getConnectVehicle && startCalibSensor)
	{
		app_adc_infor.startCalibSensor = true;
	}
	else
	{
		app_adc_infor.startCalibSensor = false;
	}
	
	// Start sensor
	if (getConnectVehicle && !startCalibSensor && startSensor)
	{
		app_adc_infor.startSensor = true;
		if (app_slave_infor.isStartSensorFlag)
		{
			isStartSensorFlag = 1;
		}
	}
	if (getConnectVehicle && !startSensor)
	{
		app_adc_infor.startSensor = false;
		if (!app_slave_infor.isStartSensorFlag)
		{
			isStartSensorFlag = 0;
		}
	}
	// Stop Sensor
	// To Do
	/*
		Processing Motor
	*/	
	// Start calib motor (need: check if sensor is calib or not before start calib and start motor)
	if (getConnectVehicle && startCalibMotor)
	{
		app_adc_infor.startCalibMotor = true;
//			isOnMotorFlag				 = 1;
	}
	if (app_slave_infor.motorCalibFlag == 2)
	{
		app_adc_infor.startCalibMotor = true;
	}
	// Start motor
	if (getConnectVehicle && !startCalibSensor && startMotor)
	{
		app_adc_infor.startCalibMotor = false;
		app_adc_infor.startMotor = true;
		isOnMotorFlag				 = 1;
	}

	if (getStateResponse && startSensor)  //
	{
		isStateResponseFlag = 1;
		isSendFeedbackFlagState = true;
	}
	osDelay(10);
}
void sensorFeedback(APPLICATION_SLAVE_INFO *slaveInfo, uint8_t *sensorFlag)
{
	if (slaveInfo->vertVelCalibFlag == 2)
	{
		*sensorFlag = 8;
	}
	else if (slaveInfo->vertVelCalibFlag == 1)
	{
		*sensorFlag = 7;
	}
	else if (slaveInfo->ms5611CalibFlag == 2)
	{
		*sensorFlag = 6;
	}
	else if (slaveInfo->ms5611CalibFlag == 1)
	{
		*sensorFlag = 5;
	}
	else if (slaveInfo->hmc5883lCalibFlag == 2)
	{
		*sensorFlag = 4;
	}
	else if (slaveInfo->hmc5883lCalibFlag == 1)
	{
		*sensorFlag = 3;
	}
	else if (slaveInfo->mpu6050CalibFlag == 2)
	{
		*sensorFlag = 2;
	}
	else if (slaveInfo->mpu6050CalibFlag == 1)
	{
		*sensorFlag = 1;
	}
	else
	{
		*sensorFlag = 0;
	}
}
