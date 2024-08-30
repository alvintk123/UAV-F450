 
#ifndef _USER_INFOR_THREAD_H_
#define _USER_INFOR_THREAD_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include "protocol.h"
#include "ms5611.h"
#include "ControlMotor.h"

#define SLAVE_BOARD 

// extern declare

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;


typedef struct APPL_INFO
{
    uint8_t  Is_upgrade;
    uint32_t Read_size;
    uint32_t Read_CRC32;
    uint32_t Cal_CRC32;
}APPLICATION_FLASH_INFO;

typedef struct ADC_INFO
{
		float 	thrustControl;
		float 	yawControl;
		float 	pitchControl;
		float 	rollControl;
		float 	PtermV;
		float 	ItermV;
		float 	DtermV;
	
		float 	PtermY;
		float 	ItermY;
		float 	DtermY;
	
		float 	PtermP;
		float 	ItermP;
		float 	DtermP;
	
		float 	PtermR;
		float 	ItermR;
		float 	DtermR;
		
		bool 		isOnMotor;
//		bool 		isSaveCalibData;
//		bool		isLoadCalibData;
//		bool		isSaveHMC5883;
//		bool		isLoadHMC5883;
	
		bool		isTunePID;
		bool    isTuneThrottle;
		// App
		bool    startCalibMotor;
		bool 		startMotor;
		bool    startCalibSensor;
		bool    startSensor;
		uint8_t status;
}APPLICATION_ADC_INFO;

typedef struct SLAVE_INFO
{
//		float   Ax;
//		float   Ay;
//		float   Az;
//		float 	PitchAngleRaw;
//		float 	RollAngleRaw;
//		float 	YawAngleRaw;
		
//		float   deltaTImu;
//		float		deltaTMain;
	
		float 	rollValue;
		float 	pitchValue;
		float 	thrustValue;
		float 	yawValue;
		float 	controlVel;
		float 	realVel;  // VertVelAfter
		float 	errorVel;
		float 	altitude; // AltitudeAfterCompKal
		float 	PitchAngle; // (deg)
		float 	RollAngle;	// (deg)
		float 	YawAngleLevel;
		
		float 	PitchRate; // (deg)
		float 	RollRate;	// (deg)
		float 	YawRate;
		
//		float   pitchRateOffset;
//		float   rollRateOffset;
//		float   yawRateOffset;
	
//		float 	inputThrottle1;
//		float 	inputThrottle2;
//		float 	inputThrottle3;
//		float 	inputThrottle4;
//		float 	ErrorVerticalVelocity;
//		float 	PrevItermVerticalVelocity;
//		float 	PTerm;
//		float 	DTerm;
//		float 	inputThrottleTemp;
//		
//		float 	ErrorYawRate;
//		float 	PrevItermYawRate;
//		float 	PTermYawRate;
//		float 	DTermYawRate;
//		float 	inputYawTemp;
//		
//		
//		float 	PTermCoefV;
//		float 	ITermCoefV;
//		float		DTermCoefV;
//		
//		float 	PTermCoefY;
//		float 	ITermCoefY;
//		float		DTermCoefY;
//		
//		float 	PTermCoefP;
//		float 	ITermCoefP;
//		float		DTermCoefP;
//		
//		float 	PTermCoefR;
//		float 	ITermCoefR;
//		float		DTermCoefR;
		
		// Save Calibdata
//		bool		isConfirmSaveHMC5883;
//		bool 		isConfirmLoadHMC5883;
		
		// App
		bool			 isConnectFlag;
		bool       isStartSensorFlag;
		uint8_t    motorCalibFlag;
		uint8_t    mpu6050CalibFlag;
		uint8_t    hmc5883lCalibFlag;
		uint8_t    ms5611CalibFlag;
		uint8_t 	 vertVelCalibFlag;
		
		uint8_t status;
}APPLICATION_SLAVE_INFO;

typedef struct SLAVE_PID
{
		float 	ErrorVerticalVelocity;
		float 	PrevItermVerticalVelocity;
		float 	PTerm;
		float 	DTerm;
		float 	inputThrottleTemp;
		uint8_t status;
}APPLICATION_SLAVE_PID;


void system_resetMCU(void);

// function declare
void Infor_Initization(void);
void Infor_Processing(void);
void Infor_SetBuzzerBeep(uint8_t timeBeep, uint16_t timeDelay);


#endif /*_USER_INFOR_THREAD_H_*/
