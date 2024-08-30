//#ifndef BMP180_H
//#define BMP180_H

//#include "stm32f4xx_hal.h"
//#include "stm32f4xx.h"
//#include "stdint.h"

///* Default I2C pin */
//#ifndef BMP180_I2C
//#define BMP180_I2C					I2C1
////#define BMP180_I2C_PINSPACK			TM_I2C_PinsPack_1
//#endif

///* Default I2C speed */
//#ifndef BMP180_I2C_SPEED
//#define BMP180_I2C_SPEED			100000
//#endif

///* BMP180 I2C address */
//#ifndef BMP180_I2C_ADDRESS
//#define BMP180_I2C_ADDRESS			0xEE
//#endif

///* Registers */
//#define	BMP180_REGISTER_CONTROL 	0xF4
//#define	BMP180_REGISTER_RESULT 		0xF6
//#define BMP180_REGISTER_EEPROM		0xAA

///* Commands */
//#define	BMP180_COMMAND_TEMPERATURE 	0x2E
//#define	BMP180_COMMAND_PRESSURE_0 	0x34
//#define	BMP180_COMMAND_PRESSURE_1 	0x74
//#define	BMP180_COMMAND_PRESSURE_2 	0xB4
//#define	BMP180_COMMAND_PRESSURE_3 	0xF4

///* Minimum waiting delay, in microseconds */
//#define BMP180_TEMPERATURE_DELAY	4500
//#define BMP180_PRESSURE_0_DELAY		4500
//#define BMP180_PRESSURE_1_DELAY		7500
//#define BMP180_PRESSURE_2_DELAY		13000
//#define BMP180_PRESSURE_3_DELAY		25000

///**
// * @}
// */
// 
///**
// * @defgroup TM_BMP180_Typedefs
// * @brief    Library Typedefs
// * @{
// */

///**
// * @brief  BMP180 result enumerations	
// */
//typedef enum {
//	TM_BMP180_Result_Ok = 0x00,            /*!< Everything OK */
//	TM_BMP180_Result_DeviceNotConnected,   /*!< Device is not connected to I2C */
//	TM_BMP180_Result_LibraryNotInitialized /*!< Library is not initialized */
//} TM_BMP180_Result_t;

///**
// * @brief  Options for oversampling settings
// * @note   This settings differs in samples for one result 
// *         and sample time for one result
// */
//typedef enum {
//	TM_BMP180_Oversampling_UltraLowPower = 0x00,      /*!< 1 sample for result */
//	TM_BMP180_Oversampling_Standard = 0x01,           /*!< 2 samples for result */
//	TM_BMP180_Oversampling_HighResolution = 0x02,     /*!< 3 samples for result */
//	TM_BMP180_Oversampling_UltraHighResolution = 0x03 /*!< 4 samples for result */
//} TM_BMP180_Oversampling_t;

///**
// * @brief  BMP180 main structure		
// */
//typedef struct {
//	float Altitude;                        /*!< Calculated altitude at given read pressure */
//	uint32_t Pressure;                     /*!< Pressure in pascals */
//	float Temperature;                     /*!< Temperature in degrees */
//	uint16_t Delay;                        /*!< Number of microseconds, that sensor needs to calculate data that you request to */
//	TM_BMP180_Oversampling_t Oversampling; /*!< Oversampling for pressure calculation */
//} TM_BMP180_t;

//typedef struct 
//{
//		uint8_t data_config[22];
//		uint8_t data_tem[2];
//		uint8_t data_pre[3];
//}BMP180_GET;

//typedef struct 
//{
//		int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
//		uint16_t AC4, AC5, AC6, UT;
//		int32_t X1, X2, X3, B3, B5, B6, T, p;
//		uint32_t B4, B7, UP;
//		float Altitude;                        /*!< Calculated altitude at given read pressure */
//		uint32_t Pressure;                     /*!< Pressure in pascals */
//		float Temperature;                     /*!< Temperature in degrees */

//}BMP180_DATA;

//uint8_t TM_BMP180_Init(TM_BMP180_t* BMP180_Data);
//uint8_t TM_BMP180_StartTemperature(TM_BMP180_t* BMP180_Data);
//uint8_t TM_BMP180_ReadTemperature(TM_BMP180_t* BMP180_Data);
//uint8_t TM_BMP180_StartPressure(TM_BMP180_t* BMP180_Data, TM_BMP180_Oversampling_t Oversampling);
//uint8_t TM_BMP180_ReadPressure(TM_BMP180_t* BMP180_Data);
//uint32_t TM_BMP180_GetPressureAtSeaLevel(uint32_t pressure, float altitude);

////user add function
//void HAL_I2C_Master_Write_ReadIT(I2C_HandleTypeDef *hi2c, uint8_t SlaveAddress, uint8_t *pDataSend, uint16_t SizeSend, uint32_t TimeoutSend, uint8_t *pDataReceive, uint16_t SizeReceive);
//void Caculator_Default(void);

//#endif /*_BMP180_H_*/
