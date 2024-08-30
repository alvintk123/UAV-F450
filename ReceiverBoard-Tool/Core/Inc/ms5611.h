
#ifndef _MS5611_H_
#define _MS5611_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stdint.h"
#include "cmsis_os2.h"
#include "mpu6050.h"

#define MS5611_ADDRESS						0x77
#define MS5611_CMD_READ_ADC       0x00
#define MS5611_CMD_RESET          0x1E
#define MS5611_CMD_CONVERT_D1     0x40
#define MS5611_CMD_CONVERT_D2     0x50

#define MS5611_CONVERSION_OSR_256  1
#define MS5611_CONVERSION_OSR_512  2
#define MS5611_CONVERSION_OSR_1024 3
#define MS5611_CONVERSION_OSR_2048 5
#define MS5611_CONVERSION_OSR_4096 10

#define MS5611_CMD_READ_PROM_C0 0xA0
#define MS5611_CMD_READ_PROM_C1 0xA2
#define MS5611_CMD_READ_PROM_C2 0xA4
#define MS5611_CMD_READ_PROM_C3 0xA6
#define MS5611_CMD_READ_PROM_C4 0xA8
#define MS5611_CMD_READ_PROM_C5 0xAA
#define MS5611_CMD_READ_PROM_C6 0xAC
#define MS5611_CMD_READ_PROM_C7 0xAE

#define MS5611_PRESSURE_OSR_256 0x40
#define MS5611_PRESSURE_OSR_512 0x42
#define MS5611_PRESSURE_OSR_1024 0x44
#define MS5611_PRESSURE_OSR_2048 0x46
#define MS5611_PRESSURE_OSR_4096 0x48

#define MS5611_TEMP_OSR_256 0x50
#define MS5611_TEMP_OSR_512 0x52
#define MS5611_TEMP_OSR_1024 0x54
#define MS5611_TEMP_OSR_2048 0x56
#define MS5611_TEMP_OSR_4096 0x58

typedef enum OSR {
	OSR_256, // 0
	OSR_512,
	OSR_1024,
	OSR_2048,
	OSR_4096
}OSR;

typedef struct
{
	uint16_t C[8];
	uint32_t DigitalPressure_D1;
	uint32_t DigitalTemperature_D2;
	int32_t dT;
	int32_t TEMP;
	int64_t OFF;
	int64_t SENS;
	int32_t P; // (Pa)

	float Alt_Offset; // offset for Alt_Filt
	float Alt; // (m)
	float Alt_Filt; // (m)

	int64_t OFF2;
	int64_t SENS2;
	int32_t T2; // T2m *0.01 = degree C

	// use for complementary filter
	float 		PreAlt;
	float 		FilterCoef;
	bool 			isStartFilter;
	bool 			isCalibAlt;
}MS5611_t;


void MS5611_Reset(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611);
void MS5611_ReadProm(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611);

void MS5611_RequestTemperature(I2C_HandleTypeDef *I2Cx, OSR osr);
void MS5611_RequestPressure(I2C_HandleTypeDef *I2Cx, OSR osr);

void MS5611_ReadTemperature(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611);
void MS5611_ReadPressure(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611);

void MS5611_CalculateTemperature(MS5611_t *MS5611);
void MS5611_CalculatePressure(MS5611_t *MS5611);

float MS5611_getAltitude1(float pressure);
float MS5611_getAltitude2(float pressure , float temperature);

void MS5611_CalibAltitude(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611, int* alt_cnt);
void MS5611_readAltitude(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611, int* alt_cnt, int isOffset);

void MS5611_Start(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611, int* alt_cnt);
/*
	
*/
void MS5611_GetAltitudeVerticalVelocity(I2C_HandleTypeDef *I2C_mpu, I2C_HandleTypeDef *I2C_ms, GY86_MPU6050_t *mpu, MS5611_t *MS5611, float verticalState[2][1], float P[2][2], int* alt_cnt, int isOffset, float dt);
//typedef struct 
//{
//	uint16_t Calib[7];

//}MS5611_DATA;


//void MS5611_Initial(void);
//void MS5611_Reset(void);
//void MS5611_ReadRom(void);
//void MS5611_ReadADC(void);
//void MS5611_ConvertD1(void);
//void MS5611_ConvertD2(void);


#endif /*_MS5611_H_*/
