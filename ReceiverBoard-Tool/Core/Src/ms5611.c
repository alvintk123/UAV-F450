
#include "ms5611.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "math.h"

uint8_t MS5611_rx_buf[12];
uint8_t MS5611_rx_temp[3];
uint8_t MS5611_rx_press[3];
uint8_t MS5611_tx;
uint8_t MS5611_rx;

MS5611_t ms5611_1;

int samples_ms = 0;

float oldAltitude = 0;
uint32_t time1 = 0;
uint32_t time2 = 0;
extern verticalState 						vertState;

void MS5611_Start(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611, int* alt_cnt)
{
	MS5611_Reset(I2Cx, MS5611);
	MS5611_ReadProm(I2Cx, MS5611);
	osDelay(1000);
//	MS5611_CalibAltitude(I2Cx, MS5611, alt_cnt);
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
//	osDelay(1000);
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
//	osDelay(1000);

}
void MS5611_Reset(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611)
{
	MS5611_tx = MS5611_CMD_RESET;
	while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1 , &MS5611_tx, 1, 100) != HAL_OK)
	{}
//	HAL_Delay(10);
	//For Temperature > 20 Celsius
	MS5611->T2 = 0;
	MS5611->OFF2 = 0;
	MS5611->SENS2 = 0;
	MS5611->isStartFilter = false;
	MS5611->FilterCoef = 0.95;
}

void MS5611_ReadProm(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611)
{
		MS5611_tx = MS5611_CMD_READ_PROM_C0;
		while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100) != HAL_OK){};
		while(HAL_I2C_Master_Receive(I2Cx, MS5611_ADDRESS << 1 , MS5611_rx_buf, 2, 100) != HAL_OK){};
//		while(HAL_I2C_Mem_Read(I2Cx, MS5611_ADDRESS <<1, MS5611_CMD_READ_PROM_C0, 1, MS5611_rx_buf, 2, 100)!= HAL_OK){};
		MS5611->C[0] = (uint16_t)(MS5611_rx_buf[0] << 8 | MS5611_rx_buf[1]);

		MS5611_tx = MS5611_CMD_READ_PROM_C1;
		while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100) != HAL_OK){};
		while(HAL_I2C_Master_Receive(I2Cx, MS5611_ADDRESS << 1 , MS5611_rx_buf, 2, 100) != HAL_OK){};
//		while(HAL_I2C_Mem_Read(I2Cx, MS5611_ADDRESS <<1, MS5611_CMD_READ_PROM_C1, 1, MS5611_rx_buf, 2, 100)!= HAL_OK){};
		MS5611->C[1] = (uint16_t) (MS5611_rx_buf[0] << 8 | MS5611_rx_buf[1]);

		MS5611_tx = MS5611_CMD_READ_PROM_C2;
		while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100) != HAL_OK){};
		while(HAL_I2C_Master_Receive(I2Cx, MS5611_ADDRESS << 1 , MS5611_rx_buf, 2, 100) != HAL_OK){};
//		while(HAL_I2C_Mem_Read(I2Cx, MS5611_ADDRESS <<1, MS5611_CMD_READ_PROM_C2, 1, MS5611_rx_buf, 2, 100)!= HAL_OK){};
		MS5611->C[2] =(uint16_t) (MS5611_rx_buf[0] << 8 | MS5611_rx_buf[1]);

		MS5611_tx = MS5611_CMD_READ_PROM_C3;
		while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100) != HAL_OK){};
		while(HAL_I2C_Master_Receive(I2Cx, MS5611_ADDRESS << 1 , MS5611_rx_buf, 2, 100) != HAL_OK){};
//		while(HAL_I2C_Mem_Read(I2Cx, MS5611_ADDRESS <<1, MS5611_CMD_READ_PROM_C3, 1, MS5611_rx_buf, 2, 100)!= HAL_OK){};
		MS5611->C[3] = (uint16_t)(MS5611_rx_buf[0] << 8 | MS5611_rx_buf[1]);

		MS5611_tx = MS5611_CMD_READ_PROM_C4;
		while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100) != HAL_OK){};
		while(HAL_I2C_Master_Receive(I2Cx, MS5611_ADDRESS << 1 , MS5611_rx_buf, 2, 100) != HAL_OK){};
//		while(HAL_I2C_Mem_Read(I2Cx, MS5611_ADDRESS <<1, MS5611_CMD_READ_PROM_C4, 1, MS5611_rx_buf, 2, 100)!= HAL_OK){};
		MS5611->C[4] = (uint16_t)(MS5611_rx_buf[0] << 8 | MS5611_rx_buf[1]);

		MS5611_tx = MS5611_CMD_READ_PROM_C5;
		while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100) != HAL_OK){};
		while(HAL_I2C_Master_Receive(I2Cx, MS5611_ADDRESS << 1 , MS5611_rx_buf, 2, 100) != HAL_OK){};
//		while(HAL_I2C_Mem_Read(I2Cx, MS5611_ADDRESS <<1, MS5611_CMD_READ_PROM_C5, 1, MS5611_rx_buf, 2, 100)!= HAL_OK){};
		MS5611->C[5] = (uint16_t)(MS5611_rx_buf[0] << 8 | MS5611_rx_buf[1]);
			
		MS5611_tx = MS5611_CMD_READ_PROM_C6;
		while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100) != HAL_OK){};
		while(HAL_I2C_Master_Receive(I2Cx, MS5611_ADDRESS << 1 , MS5611_rx_buf, 2, 100) != HAL_OK){};
//		while(HAL_I2C_Mem_Read(I2Cx, MS5611_ADDRESS <<1, MS5611_CMD_READ_PROM_C6, 1, MS5611_rx_buf, 2, 100)!= HAL_OK){};
		MS5611->C[6] = (uint16_t)(MS5611_rx_buf[0] << 8 | MS5611_rx_buf[1]);

		MS5611_tx = MS5611_CMD_READ_PROM_C7;
		while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100) != HAL_OK){};
		while(HAL_I2C_Master_Receive(I2Cx, MS5611_ADDRESS << 1 , MS5611_rx_buf, 2, 100) != HAL_OK){};
//		while(HAL_I2C_Mem_Read(I2Cx, MS5611_ADDRESS <<1, MS5611_CMD_READ_PROM_C7, 1, MS5611_rx_buf, 2, 100)!= HAL_OK){};
		MS5611->C[7] = (uint16_t)(MS5611_rx_buf[0] << 8 | MS5611_rx_buf[1]);
}


void MS5611_RequestTemperature(I2C_HandleTypeDef *I2Cx, OSR osr)
{
	MS5611_tx = MS5611_TEMP_OSR_256 + (2 * osr);
	while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS <<1, &MS5611_tx, 1, 100)!= HAL_OK){};
//	HAL_I2C_Master_Transmit_IT(I2Cx, MS5611_ADDRESS <<1, &MS5611_tx, 1);
}

void MS5611_RequestPressure(I2C_HandleTypeDef *I2Cx, OSR osr)
{
	MS5611_tx = MS5611_PRESSURE_OSR_256 + (2 * osr);
	while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100)!= HAL_OK){};
//	HAL_I2C_Master_Transmit_IT(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1);
}

void MS5611_ReadTemperature(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611)
{
	//Read ADC
	MS5611_tx = MS5611_CMD_READ_ADC;
//	while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100) != HAL_OK){};
//	while(HAL_I2C_Master_Receive(I2Cx, (MS5611_ADDRESS << 1) | 0x01, MS5611_rx_temp, 3, 100) != HAL_OK){};
	
	while(HAL_I2C_Mem_Read(I2Cx, MS5611_ADDRESS <<1 , MS5611_CMD_READ_ADC, 1, MS5611_rx_temp, 3, 100)!= HAL_OK){};
//	HAL_I2C_Mem_Read_IT(I2Cx, MS5611_ADDRESS <<1 , MS5611_CMD_READ_ADC, 1, MS5611_rx_temp, 3);

	MS5611->DigitalTemperature_D2 = (uint32_t)((MS5611_rx_temp[0] << 16) | (MS5611_rx_temp[1] << 8) | MS5611_rx_temp[2]);
}

void MS5611_ReadPressure(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611)
{
	//Read ADC
	MS5611_tx = MS5611_CMD_READ_ADC;
//	while(HAL_I2C_Master_Transmit(I2Cx, MS5611_ADDRESS << 1, &MS5611_tx, 1, 100) != HAL_OK){};
//	while(HAL_I2C_Master_Receive(I2Cx, (MS5611_ADDRESS << 1) | 0x01, MS5611_rx_press, 3, 100) != HAL_OK){};
	
	while(HAL_I2C_Mem_Read(I2Cx, MS5611_ADDRESS <<1, MS5611_CMD_READ_ADC, 1, MS5611_rx_press, 3, 100)!= HAL_OK){};
//	HAL_I2C_Mem_Read_IT(I2Cx, MS5611_ADDRESS <<1, MS5611_CMD_READ_ADC, 1, MS5611_rx_press, 3);

	MS5611->DigitalPressure_D1 = (uint32_t)(MS5611_rx_press[0] << 16 | MS5611_rx_press[1] << 8 | MS5611_rx_press[2]);
}

void MS5611_CalculateTemperature(MS5611_t *MS5611)
{
//	MS5611->dT = MS5611->C[5];
//	MS5611->dT <<= 8; //Calculated up to C5 * 2^8
//	MS5611->dT *= -1; //Apply negative sign
//	MS5611->dT += MS5611->DigitalTemperature_D2; // = D2 - C5 * 2^8
//	
	
//	MS5611->TEMP = (MS5611->dT * MS5611->C[6]);
//	MS5611->TEMP >>= 23; // Calculated up to dT * C6 / 2^23
//	MS5611->TEMP += 2000;
	
	
//	MS5611->dT = MS5611->DigitalTemperature_D2 - (MS5611->C[5]<<8);
//	MS5611->TEMP = 2000 + (MS5611->dT*MS5611->C[6]>>23);
	
	MS5611->dT = MS5611->DigitalTemperature_D2 - ((uint32_t)MS5611->C[5] *	256);
	MS5611->TEMP = 2000 + ((int64_t)MS5611->dT * MS5611->C[6])/8388608;
	
	if (MS5611->TEMP < 2000)
	{
//		MS5611->TEMP = MS5611->TEMP - ((MS5611->dT*MS5611->dT)>>31); // TEMP = TEMP - dT^2/2^31
		MS5611->TEMP = MS5611->TEMP - ((MS5611->dT*MS5611->dT) / pow(2, 31)); // TEMP = TEMP - dT^2/2^31
	}
}

void MS5611_CalculatePressure(MS5611_t *MS5611)
{
//	MS5611->OFF = MS5611->C[2];
//	MS5611->OFF <<= 16; //Calculated up to C2 * 2^16
//	MS5611->OFF += (MS5611->C[4] * MS5611->dT) >> 7;


//	MS5611->SENS = MS5611->C[1];
//	MS5611->SENS <<= 15; // Calculated up to C1 * 2^15
//	MS5611->SENS += ((MS5611->C[3] * MS5611->dT) >>8);
//	if (MS5611->TEMP<2000)
//	{
//		MS5611->OFF2 = 5.0*(MS5611->TEMP-2000)*(MS5611->TEMP-2000)/2.0;
//		MS5611->SENS2 = 5.0*(MS5611->TEMP-2000)*(MS5611->TEMP-2000)/(2.0*2.0);
//		if (MS5611->TEMP<1500)
//		{
//			MS5611->OFF2 += 7.0*(MS5611->TEMP+1500)*(MS5611->TEMP+1500);
//			MS5611->SENS2 += 11.0*(MS5611->TEMP+1500)*(MS5611->TEMP+1500)/2.0;
//		}
//		MS5611->OFF -= MS5611->OFF2;
//		MS5611->SENS -= MS5611->SENS2;
//	}
//	
//	MS5611->P = ((MS5611->DigitalPressure_D1 * MS5611->SENS) / pow(2, 21) - MS5611->OFF) / pow(2, 15);
	MS5611->OFF = (int64_t)MS5611->C[2] * 65536 + (int64_t)MS5611->C[4] * MS5611->dT / 128 ;
	MS5611->SENS = (int64_t)MS5611->C[1] * 32768 + (int64_t) MS5611->C[3] * MS5611->dT /256 ;
	
	if (MS5611->TEMP<2000)
	{
		MS5611->OFF2 = 5*(MS5611->TEMP-2000)*(MS5611->TEMP-2000)/2;
		MS5611->SENS2 = 5*(MS5611->TEMP-2000)*(MS5611->TEMP-2000)/(2*2);
		if (MS5611->TEMP<-1500)
		{
			MS5611->OFF2 += (7*(MS5611->TEMP+1500)*(MS5611->TEMP+1500));
			MS5611->SENS2 += (11*(MS5611->TEMP+1500)*(MS5611->TEMP+1500)/2);
		}
		MS5611->OFF -= MS5611->OFF2;
		MS5611->SENS -= MS5611->SENS2;
	}
	
	MS5611->P = (((MS5611->DigitalPressure_D1 * MS5611->SENS) / 2097152 - MS5611->OFF) / 32768);
	
}

#define SEA_PRESSURE 101325.0f

float MS5611_getAltitude1(float pressure) //No temperature correction.
{
	return (1.0f - powf((pressure / SEA_PRESSURE), 0.1902226f)) * 44307.69396f; //145366.45f * 0.3048f = 44307.69396f;
}

float MS5611_getAltitude2(float pressure, float temperature) //Get Altitude with temperature correction.
{
	return (1.0f - powf((pressure / SEA_PRESSURE), 0.1902226f)) * (temperature + 273.15f) / 0.0065f;
}
void MS5611_CalibAltitude(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611, int* alt_cnt)
{
	
	float altitude_offset = 0;
	samples_ms = 0;
	while (samples_ms <2001)
	{
		MS5611_readAltitude(I2Cx, MS5611, alt_cnt, 0);
		osDelay(10);
		if (*alt_cnt == 1)
		{
			// drop the first times because don't have value of altitude
			samples_ms ++;
		}
	}
	samples_ms = 0;
	*alt_cnt = 0;
	while (samples_ms <2001)
	{
		MS5611_readAltitude(I2Cx, MS5611, alt_cnt, 0);
		osDelay(10);
		if (*alt_cnt == 1)
		{
			// drop the first times because don't have value of altitude
			if (samples_ms > 0)
			{
//				altitude_offset += MS5611->Alt_Filt;
//				altitude_offset += MS5611->Alt;
				altitude_offset = MS5611->Alt;
			}
			samples_ms ++;
		}
	}
		
//	MS5611->Alt_Offset = (float) (altitude_offset / 2000.0);
	MS5611->Alt_Offset = (float) (altitude_offset);
	
	MS5611->isCalibAlt = true;
	// Reset count variable
	*alt_cnt = 0;
}
void MS5611_readAltitude(I2C_HandleTypeDef *I2Cx, MS5611_t *MS5611, int* alt_cnt, int isOffset)
{
//		MS5611_RequestTemperature(I2Cx, OSR_4096);
//		MS5611_ReadTemperature(I2Cx, MS5611);
//		MS5611_CalculateTemperature(MS5611);
//		MS5611_RequestPressure(I2Cx, OSR_4096);
//		MS5611_ReadPressure(I2Cx, MS5611);
//		MS5611_CalculatePressure(MS5611);
//		MS5611->Alt = MS5611_getAltitude2((float)MS5611->P,(float)MS5611->TEMP/100.0f);
//	
		if (*alt_cnt == 0)
		{
			MS5611_RequestTemperature(I2Cx, OSR_2048);
			*alt_cnt = 1;
		}
		else if (*alt_cnt == 1)
		{
			MS5611_ReadTemperature(I2Cx, MS5611);
			MS5611_CalculateTemperature(MS5611);
			MS5611_RequestPressure(I2Cx, OSR_2048);
			*alt_cnt = 2;
		}
		else
		{
			MS5611_ReadPressure(I2Cx, MS5611);
			MS5611_CalculatePressure(MS5611);
			MS5611_RequestTemperature(I2Cx, OSR_2048);
			MS5611->Alt = MS5611_getAltitude1((float)MS5611->P);
			MS5611->Alt_Filt = MS5611_getAltitude2((float)MS5611->P,(float)MS5611->TEMP/100.0f);
			if (isOffset)
			{
				MS5611->Alt -= MS5611->Alt_Offset;
			}
			*alt_cnt = 1;
		}
		
}

void MS5611_GetAltitudeVerticalVelocity(I2C_HandleTypeDef *I2C_mpu, I2C_HandleTypeDef *I2C_ms, GY86_MPU6050_t *mpu, MS5611_t *MS5611, float verticalState[2][1], float P[2][2], int* alt_cnt, int isOffset, float dt)
{
	MS5611_readAltitude(I2C_ms, MS5611, alt_cnt, 1);
	osDelay(10);
	GetVerticalVelocity(I2C_mpu, mpu, dt);
//	KalMan_2D(verticalState, P, mpu->AccZInertial, MS5611->Alt_Filt, dt); // 10ms = 0.01s
	/*
	Start complementary filter
	*/
//	time1 = osKernelGetTickCount();
//	uint32_t deltaTime = time1 - time2;
//	if (ms5611_1.Alt-oldAltitude >= 10) 
//	{
//		if (deltaTime*vertState.velocity > ms5611_1.Alt-oldAltitude)
//		{
//			oldAltitude = ms5611_1.Alt;
//		}
//		else
//		{
//			ms5611_1.Alt = oldAltitude;
//		}
//	}
//	else
//	{
//		oldAltitude = ms5611_1.Alt;
//	}
	if (!MS5611->isStartFilter)
	{
		MS5611->isStartFilter = true;
		MS5611->PreAlt = MS5611->Alt;
	}
	MS5611->Alt = MS5611->FilterCoef * MS5611->PreAlt + (1 - MS5611->FilterCoef) * MS5611->Alt;
	MS5611->PreAlt = MS5611->Alt;
	KalMan_2D(verticalState, P, mpu->AccZInertial, MS5611->Alt, dt); // 10ms = 0.01s
	
}

//MS5611_DATA ms5611;


//void MS5611_ReadCalibarationFromPROM(void)
//{
//		uint8_t dataIn[1];
//		uint8_t dataOut[14];

//		dataIn[0]= MS5611_CMD_READ_PROM;
//		while(HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,dataIn,1,4)!=HAL_OK)
//		 {}	
//		while(HAL_I2C_Master_Receive_IT(&hi2c1, MS5611_ADDRESS, dataOut, 14) != HAL_OK) 
//		 {}
//		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
//		memcpy(ms5611.Calib, dataOut, sizeof(dataOut));
//}




