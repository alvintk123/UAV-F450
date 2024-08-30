//#include "bmp180.h"
//#include "main.h"
//#include <stdbool.h>
//#include <stdint.h>
//#include <stdio.h>
//#include "math.h"

//extern I2C_HandleTypeDef hi2c1;

//extern BMP180_DATA							bmp180;
//extern BMP180_GET								bmp180_get;

//int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
//uint16_t AC4, AC5, AC6, UT;
///* OK */
//int32_t X1, X2, X3, B3, B5, B6, T, p;
//uint32_t B4, B7, UP;
//uint8_t lib_initialized = 0;

///* Multiple is faster than divide */
//#define BMP180_1_16     ((float) 0.0625)
//#define BMP180_1_256    ((float) 0.00390625)
//#define BMP180_1_2048   ((float) 0.00048828125)
//#define BMP180_1_4096   ((float) 0.000244140625)
//#define BMP180_1_8192   ((float) 0.0001220703125)
//#define BMP180_1_32768  ((float) 0.000030517578125)
//#define BMP180_1_65536  ((float) 0.0000152587890625)
//#define BMP180_1_101325 ((float) 0.00000986923266726)



//uint8_t TM_BMP180_Init(TM_BMP180_t* BMP180_Data) {
//	//uint8_t i = 0;
//	
//	/* Initialize I2C */ // no need init because init in main
//	//TM_I2C_Init(BMP180_I2C, BMP180_I2C_PINSPACK, BMP180_I2C_SPEED);
//	/* Test if device is connected */
////	if (!TM_I2C_IsDeviceConnected(BMP180_I2C, BMP180_I2C_ADDRESS)) {
////		/* Device is not connected */
////		return TM_BMP180_Result_DeviceNotConnected;
////	}
//		if(HAL_I2C_IsDeviceReady(&hi2c1,BMP180_I2C_ADDRESS, 3, 100) != HAL_OK)
//		{
//			return TM_BMP180_Result_DeviceNotConnected;
//		}
//		uint8_t dataSend[1];
//		dataSend[0] = BMP180_REGISTER_EEPROM;
//		HAL_I2C_Master_Write_ReadIT(&hi2c1,BMP180_I2C_ADDRESS,dataSend, sizeof(dataSend), 2, bmp180_get.data_config, 22 );
//	
//	/* Initialized OK */
//	lib_initialized = 1;
//	
//	/* Return OK */
//	return TM_BMP180_Result_Ok;
//}



//uint8_t TM_BMP180_StartTemperature(TM_BMP180_t* BMP180_Data) {
//	/* Check for library initialization */
//	if (!lib_initialized) {
//		return TM_BMP180_Result_LibraryNotInitialized;
//	}
//	/* Send to device */
//	//TM_I2C_Write(BMP180_I2C, BMP180_I2C_ADDRESS, BMP180_REGISTER_CONTROL, BMP180_COMMAND_TEMPERATURE);
//		 uint8_t datain[2];
//		 datain[0]= BMP180_REGISTER_CONTROL;
//		 datain[1]= BMP180_COMMAND_TEMPERATURE;
//		 while(HAL_I2C_Master_Transmit(&hi2c1,BMP180_I2C_ADDRESS,datain,2,4)!=HAL_OK);
//		 __HAL_I2C_CLEAR_FLAG(&hi2c1,I2C_FLAG_STOPF);
//	/* Set minimum delay */
//	BMP180_Data->Delay = BMP180_TEMPERATURE_DELAY;
//	/* Return OK */
//	return TM_BMP180_Result_Ok;
//}

//uint8_t TM_BMP180_ReadTemperature(TM_BMP180_t* BMP180_Data) {
//	
//	
//	/* Check for library initialization */
//	if (!lib_initialized) {
//		return TM_BMP180_Result_LibraryNotInitialized;
//	}
//		uint8_t dataout[1];
//		dataout[0]=BMP180_REGISTER_RESULT;
//		//dataout[1]=BMP180_REGISTER_RESULT + 1;
//		while(HAL_I2C_Master_Transmit(&hi2c1,BMP180_I2C_ADDRESS,dataout,1,4)!=HAL_OK)
//		 {}	
//		while(HAL_I2C_Master_Receive_IT(&hi2c1, BMP180_I2C_ADDRESS, bmp180_get.data_tem, 2) != HAL_OK) 
//		 {}
//		__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
//	
//	/* Get uncompensated temperature */
//	bmp180.UT = bmp180_get.data_tem[0] << 8 | bmp180_get.data_tem[1];	
//	/* Calculate true temperature */
//	bmp180.X1 = (bmp180.UT - bmp180.AC6) * bmp180.AC5 * BMP180_1_32768;
//	bmp180.X2 = bmp180.MC * 2048 / (bmp180.X1 + bmp180.MD);
//	bmp180.B5 = bmp180.X1 + bmp180.X2;
//	
//	/* Get temperature in degrees */
//	bmp180.Temperature = (bmp180.B5 + 8) / ((float)160);

//	
//	/* Return OK */
//	return TM_BMP180_Result_Ok;
//}

//uint8_t TM_BMP180_StartPressure(TM_BMP180_t* BMP180_Data, TM_BMP180_Oversampling_t Oversampling) {
//	uint8_t command;
//	
//	/* Check for library initialization */
//	if (!lib_initialized) {
//		return TM_BMP180_Result_LibraryNotInitialized;
//	}
//	
//	switch (Oversampling) {
//		case TM_BMP180_Oversampling_UltraLowPower :
//			command = BMP180_COMMAND_PRESSURE_0;
//			BMP180_Data->Delay = BMP180_PRESSURE_0_DELAY;
//			break;
//		case TM_BMP180_Oversampling_Standard:
//			command = BMP180_COMMAND_PRESSURE_1;
//			BMP180_Data->Delay = BMP180_PRESSURE_1_DELAY;
//			break;
//		case TM_BMP180_Oversampling_HighResolution:
//			command = BMP180_COMMAND_PRESSURE_2;
//			BMP180_Data->Delay = BMP180_PRESSURE_2_DELAY;
//			break;
//		case TM_BMP180_Oversampling_UltraHighResolution:
//			command = BMP180_COMMAND_PRESSURE_3;
//			BMP180_Data->Delay = BMP180_PRESSURE_3_DELAY;
//			break;
//		default:
//			command = BMP180_COMMAND_PRESSURE_0;
//			BMP180_Data->Delay = BMP180_PRESSURE_0_DELAY;
//			break;
//	}
//		uint8_t datain[2];
//		datain[0]= BMP180_REGISTER_CONTROL;
//		datain[1]= command;
//		while(HAL_I2C_Master_Transmit(&hi2c1,BMP180_I2C_ADDRESS,datain,2,4)!=HAL_OK);
//		__HAL_I2C_CLEAR_FLAG(&hi2c1,I2C_FLAG_STOPF);
//	/* Save selected oversampling */
//	BMP180_Data->Oversampling = Oversampling;
//	/* Return OK */
//	return TM_BMP180_Result_Ok;
//}

//uint8_t TM_BMP180_ReadPressure(TM_BMP180_t* BMP180_Data) {
//	
//	/* Check for library initialization */
//	if (!lib_initialized) {
//		return TM_BMP180_Result_LibraryNotInitialized;
//	}
//	
//		uint8_t dataout[1];
//	
//		dataout[0]=BMP180_REGISTER_RESULT;
//		while(HAL_I2C_Master_Transmit(&hi2c1,BMP180_I2C_ADDRESS,dataout,1,4)!=HAL_OK)
//		 {}	
//		while(HAL_I2C_Master_Receive_IT(&hi2c1, BMP180_I2C_ADDRESS, bmp180_get.data_pre, 3) != HAL_OK) 
//		 {}
//				__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
///* Get uncompensated pressure */
//	bmp180.UP = (bmp180_get.data_pre[0] << 16 | bmp180_get.data_pre[1] << 8 | bmp180_get.data_pre[2]) >> (8 - (uint8_t)BMP180_Data->Oversampling);

//	/* Calculate true pressure */
//	bmp180.B6 = bmp180.B5 - 4000;
//	bmp180.X1 = (bmp180.B2 * (bmp180.B6 * bmp180.B6 * BMP180_1_4096)) * BMP180_1_2048;
//	bmp180.X2 = bmp180.AC2 * bmp180.B6 * BMP180_1_2048;
//	bmp180.X3 = bmp180.X1 + bmp180.X2;
//	bmp180.B3 = (((bmp180.AC1 * 4 + bmp180.X3) << (uint8_t)BMP180_Data->Oversampling) + 2) * 0.25;
//	bmp180.X1 = bmp180.AC3 * bmp180.B6 * BMP180_1_8192;
//	bmp180.X2 = (bmp180.B1 * (bmp180.B6 * bmp180.B6 * BMP180_1_4096)) * BMP180_1_65536;
//	bmp180.X3 = ((bmp180.X1 + bmp180.X2) + 2) * 0.25;
//	bmp180.B4 = bmp180.AC4 * (uint32_t)(bmp180.X3 + 32768) * BMP180_1_32768;
//	bmp180.B7 = ((uint32_t)bmp180.UP - bmp180.B3) * (50000 >> (uint8_t)BMP180_Data->Oversampling);
//	if (bmp180.B7 < 0x80000000) {
//		bmp180.p = (bmp180.B7 * 2) / bmp180.B4;
//	} else {
//		bmp180.p = (bmp180.B7 / bmp180.B4) * 2;
//	}
//	bmp180.X1 = ((float)bmp180.p * BMP180_1_256) * ((float)bmp180.p * BMP180_1_256);
//	bmp180.X1 = (bmp180.X1 * 3038) * BMP180_1_65536;
//	bmp180.X2 = (-7357 * bmp180.p) * BMP180_1_65536;
//	bmp180.p = bmp180.p + (bmp180.X1 + bmp180.X2 + 3791) * BMP180_1_16;
//	
//	/* Save pressure */
//	bmp180.Pressure = bmp180.p;
//	
//	/* Calculate altitude */
//	bmp180.Altitude = (float)44330.0 * (float)((float)1.0 - (float)pow((float)bmp180.p * BMP180_1_101325, 0.19029495));
//	/* Return OK */
//	return TM_BMP180_Result_Ok;
//}
//uint32_t TM_BMP180_GetPressureAtSeaLevel(uint32_t pressure, float altitude) {
//	return (uint32_t)((float)pressure / ((float)pow(1 - (float)altitude / (float)44330, 5.255)));
//}

//void HAL_I2C_Master_Write_ReadIT(I2C_HandleTypeDef *hi2c, uint8_t SlaveAddress, uint8_t *pDataSend, uint16_t SizeSend, uint32_t TimeoutSend, uint8_t *pDataReceive, uint16_t SizeReceive)
//{
//		while(HAL_I2C_Master_Transmit(hi2c,SlaveAddress,pDataSend,SizeSend,TimeoutSend)!=HAL_OK)
//		 {}	
//		while(HAL_I2C_Master_Receive_IT(hi2c, SlaveAddress, pDataReceive, SizeReceive) != HAL_OK) 
//		 {}
//				__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_STOPF);
//}
//void Caculator_Default(void)
//{
//		uint8_t i =0 ;
//		bmp180.AC1 = (int16_t)(bmp180_get.data_config[0] << 8 | bmp180_get.data_config[1]); i += 2;
//		bmp180.AC2 = (int16_t)(bmp180_get.data_config[i] << 8 | bmp180_get.data_config[i + 1]); i += 2;
//		bmp180.AC3 = (int16_t)(bmp180_get.data_config[i] << 8 | bmp180_get.data_config[i + 1]); i += 2;
//		bmp180.AC4 = (uint16_t)(bmp180_get.data_config[i] << 8 | bmp180_get.data_config[i + 1]); i += 2;
//		bmp180.AC5 = (uint16_t)(bmp180_get.data_config[i] << 8 | bmp180_get.data_config[i + 1]); i += 2;
//		bmp180.AC6 = (uint16_t)(bmp180_get.data_config[i] << 8 | bmp180_get.data_config[i + 1]); i += 2;
//		bmp180.B1 = (int16_t)(bmp180_get.data_config[i] << 8 | bmp180_get.data_config[i + 1]); i += 2;
//		bmp180.B2 = (int16_t)(bmp180_get.data_config[i] << 8 | bmp180_get.data_config[i + 1]); i += 2;
//		bmp180.MB = (int16_t)(bmp180_get.data_config[i] << 8 | bmp180_get.data_config[i + 1]); i += 2;
//		bmp180.MC = (int16_t)(bmp180_get.data_config[i] << 8 | bmp180_get.data_config[i + 1]); i += 2;
//		bmp180.MD = (int16_t)(bmp180_get.data_config[i] << 8 | bmp180_get.data_config[i + 1]);
//}
