
#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include "stdint.h"
#include "stdbool.h"

#define PI 3.14159265358979323846f
#define TRUE 1
#define FALSE 0

#define MPU6050_I2C				I2C1
#define MPU6050_ADDRESS		0xD0


/* Register map */
#define SELF_TEST_X			0x0D
#define SELF_TEST_Y			0x0E
#define SELF_TEST_Z			0x0F
#define SELF_TEST_A			0x10
#define SMPLRT_DIV			0x19
#define CONFIG					0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define MOT_THR					0x1F
#define FIFO_EN					0x23
#define I2C_MST_CTRL		0x24
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV1_ADDR		0x28
#define I2C_SLV1_REG		0x29
#define I2C_SLV1_CTRL		0x2A
#define I2C_SLV2_ADDR		0x2B
#define I2C_SLV2_REG		0x2C
#define I2C_SLV2_CTRL		0x2D
#define I2C_SLV3_ADDR		0x2E
#define I2C_SLV3_REG		0x2F
#define I2C_SLV3_CTRL		0x30
#define I2C_SLV4_ADDR		0x31
#define I2C_SLV4_REG		0x32
#define I2C_SLV4_DO			0x33
#define I2C_SLV4_CTRL		0x34
#define I2C_SLV4_DI			0x35
#define I2C_MST_STATUS	0x36
#define INT_PIN_CFG 		0x37
#define INT_ENABLE			0x38
#define INT_STATUS 			0x3A
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40
#define TEMP_OUT_H			0x41
#define TEMP_OUT_L			0x42
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48
#define EXT_SENS_DATA_00	0x49
#define EXT_SENS_DATA_01	0x4A
#define EXT_SENS_DATA_02	0x4B
#define EXT_SENS_DATA_03	0x4C
#define EXT_SENS_DATA_04	0x4D
#define EXT_SENS_DATA_05	0x4E
#define EXT_SENS_DATA_06	0x4F
#define EXT_SENS_DATA_07	0x50
#define EXT_SENS_DATA_08	0x51
#define EXT_SENS_DATA_09	0x52
#define EXT_SENS_DATA_10	0x53
#define EXT_SENS_DATA_11	0x54
#define EXT_SENS_DATA_12	0x55
#define EXT_SENS_DATA_13	0x56
#define EXT_SENS_DATA_14	0x57
#define EXT_SENS_DATA_15	0x58
#define EXT_SENS_DATA_16	0x59
#define EXT_SENS_DATA_17	0x5A
#define EXT_SENS_DATA_18	0x5B
#define EXT_SENS_DATA_19	0x5C
#define EXT_SENS_DATA_20	0x5D
#define EXT_SENS_DATA_21	0x5E
#define EXT_SENS_DATA_22	0x5F
#define EXT_SENS_DATA_23	0x60
#define I2C_SLV0_DO			0x63
#define I2C_SLV1_DO			0x64
#define I2C_SLV2_DO			0x65
#define I2C_SLV3_DO			0x66
#define I2C_MST_DELAY_CTRL	0x67
#define I2C_SIG_PATH_RESET	0x68
#define MOT_DETECT_CTRL		0x69
#define USER_CTRL			0x6A
#define PWR_MGMT_1			0x6B
#define PWR_MGMT_2			0x6C
#define FIFO_COUNTH			0x72
#define FIFO_COUNTL 		0x73
#define FIFO_R_W 			0x74
#define WHO_AM_I			0x75

/* Gyroscope LSB sensitivity defines */
#define MPU6050_GYRO_RANGE_250		((float)131)
#define MPU6050_GYRO_RANGE_500		((float)65.5)
#define MPU6050_GYRO_RANGE_1000		((float)32.8)
#define MPU6050_GYRO_RANGE_2000		((float)16.4)

/* Accelerometer LSB sensitivity defines */
#define MPU6050_ACCEL_RANGE_2g		((float)16384)
#define MPU6050_ACCEL_RANGE_4g		((float)8192)
#define MPU6050_ACCEL_RANGE_8g		((float)4096)
#define MPU6050_ACCEL_RANGE_16g		((float)2048)

/* Maximum values for timeout flags waiting loops. These timeouts are not "time" defined
 * and are used just that application doesn't get stuck if I2C communication is corrupted.
 */
#define MPU6050_FLAG_TIMEOUT             (uint32_t)0x1000
#define MPU6050_LONG_TIMEOUT             (uint32_t)(10 * MPU6050_FLAG_TIMEOUT)


typedef struct{

	float gyroMul;		//Gyroscope raw data multiplier
	float accelMul;		//Accelerometer raw data multiplier

}MPU6050_dataStruct;

typedef enum{
	/* MPU6050 I2C success */
	MPU6050_NO_ERROR = 0,
	/* I2C error */
	MPU6050_I2C_ERROR = 1,
	/* TX error */
	MPU6050_I2C_TX_ERROR = 2,
	/* TX error */
	MPU6050_I2C_RX_ERROR = 3,

}MPU6050_errorstatus;

/* Gyroscope Full scale range options 	@gyro_scale_range */
typedef enum{

	MPU6050_GYRO_250 = 0x00,
	MPU6050_GYRO_500 = 0x08,
	MPU6050_GYRO_1000 = 0x10,
	MPU6050_GYRO_2000 = 0x18

}MPU6050_Gyro_Range;

/* Accelerometer's full scale range options		@accel_scale_range */
typedef enum{

	MPU6050_ACCEL_2g = 0x00,
	MPU6050_ACCEL_4g = 0x08,
	MPU6050_ACCEL_8g = 0x10,
	MPU6050_ACCEL_16g = 0x18
}MPU6050_Accel_Range;

/* Power management 1 	@pwr_mngt_1 */
typedef enum{

	MPU6050_INTERNAL_OSC = 0x00,
	MPU6050_PLL_X_GYRO = 0x01,
	MPU6050_PLL_Y_GYRO = 0x02,
	MPU6050_PLL_Z_GYRO = 0x03,
	MPU6050_PLL_EXT_32KHZ = 0x04,
	MPU6050_PLL_EXT_19MHZ = 0x05,
	MPU6050_STOP_CLOCK = 0x07
}MPU6050_Clock_Select;

typedef struct 
{
	
		float rollCom  ,rollAccel;
		float pitchCom, pitchAccel;
		float yawCom ,yawAccel;
		float dt;
		float val[7];
		int16_t of[7];
		int16_t Accel_Gyro[7];
		int16_t ax_calib, ay_calib, az_calib, gx_calib, gy_calib, gz_calib;
		uint32_t t_old, t_new ,temp_dt;
		uint8_t data_re[14];
		float temperature;

}MPU6050_DATA;


uint32_t MPU6050_GetTickCount(void);
void MPU6050_GetOffset(int16_t*offset);
void MPU6050_GetValue(float * value);
void MPU6050_GetAngle(void);
void MPU6050_GetRawAccelGyro(int16_t* AccelGyro);
void MPU6050_Readdata(void);
void SD_MPU6050_Init(MPU6050_Clock_Select Clock_Select, MPU6050_Gyro_Range Range_Gyro,MPU6050_Accel_Range Accel_Range );




// -------------------------------------------------------------------Khang----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define MPU6050_ 							0x68
#define MPU6050_ADDR 					MPU6050_ << 1 //Already Left Shifted
#define CONFIG_REG 						0x1A
#define GYRO_CONFIG_REG 			0x1B
#define ACCEL_CONFIG_REG 			0x1C
#define SMPLRT_DIV_REG 				0x19
#define I2C_MST_CTRL 					0x24
#define I2C_SLV0_ADDR 				0x25
#define I2C_SLV0_REG					0x26
#define I2C_SLV0_CTRL					0x27
#define INT_PIN_CFG 					0x37
#define INT_ENABLE_REG  			0x38
#define INT_STATUS_REG  			0x3A
#define ACCEL_XOUT_H_REG 			0x3B
#define TEMP_OUT_H_REG 				0x41
#define GYRO_XOUT_H_REG 			0x43
#define MAGNETIC_XOUT_H_REG 	0x49
#define USER_CTRL_REG 				0x6A
#define PWR_MGMT_1_REG  			0x6B
#define WHO_AM_I_REG 					0x75

/////////////////////////////////////
#define HMC5883L_ADDRESS              0x1E // Not Left Shifted
#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)

// MPU6050 structure
typedef struct _GY86_MPU6050_t
{
	int16_t Gyro_X_Offset_RAW;
	int16_t Gyro_Y_Offset_RAW;
	int16_t Gyro_Z_Offset_RAW;
	float rollRateOffset;
	float pitchRateOffset;
	float yawRateOffset;
	
	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	
	float rollRate;
	float pitchRate;
	float yawRate;
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	float Ax;
	float Ay;
	float Az;
	
	
	float Temperature;
	int16_t Mag_X_RAW;
	int16_t Mag_Y_RAW;
	int16_t Mag_Z_RAW;

	int16_t Mag_X_Min;
	int16_t Mag_Y_Min;
	int16_t Mag_Z_Min;

	int16_t Mag_X_Max;
	int16_t Mag_Y_Max;
	int16_t Mag_Z_Max;

	int16_t Mag_X_Offset;
	int16_t Mag_Y_Offset;
	int16_t Mag_Z_Offset;

	float Mx;
	float My;
	float Mz;
	
	float PitchAngleRaw; // (deg)
	float RollAngleRaw;	// (deg)
//	float YawAngleLevelRaw; // On level Plane (deg)
	
	float PitchAngle; // (deg)
	float RollAngle;	// (deg)
	float YawAngleLevel; // On level Plane (deg)
	float YawAngleTilte; // On tilt Plane (deg)
	
	float AccZInertial; // Acceleration follow Z inertial coordinate  (m/s^2)
	float VerticalVelocity; // (m/s)
	bool isCalibGyro;
	bool isCalibMag;
	
	// filter for gyro
	int16_t PreGyro_X_RAW;
	int16_t PreGyro_Y_RAW;
	int16_t PreGyro_Z_RAW;
	float FilterCoef;
	
} GY86_MPU6050_t;


uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t Gyro_FS, uint8_t ACC_FS, uint8_t DLPF_CFG);
void CalibMPU6050(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu);
void MPU6050_Read_Gyro_DMA(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu, int isOffset);
void MPU6050_Bypass(I2C_HandleTypeDef *I2Cx);
void MPU6050_Master(I2C_HandleTypeDef *I2Cx);
void HMC5883L_Setup(I2C_HandleTypeDef *I2Cx);
void MPU6050_Slave_Read(I2C_HandleTypeDef *I2Cx);
void CalibHMC5883L(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu);
void KalMan_1D(float *kalmanState, float *kalmanUncertainty, float kalmanInput, float kalmanMeasurement, float stDevGyro, float stdDevAccel, double dt);
void KalMan_2D(float kalmanState[2][1], float kalmanUncertainty[2][2], float kalmanInput, float kalmanMeasurement, float dt);
void MPU6050_Read_Magnetic(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu,int isOffset);
void GetRollPitchYawAngle(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu, float dt);
void GetVerticalVelocity(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu, float dt);

void MPU6050_Start(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu, uint8_t Gyro_FS, uint8_t ACC_FS, uint8_t DLPF_CFG);
void HMC5883L_Start(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu);
#endif /*_MPU6050_H_*/
