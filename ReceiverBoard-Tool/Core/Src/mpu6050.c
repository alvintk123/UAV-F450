
#include "mpu6050.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "math.h"
#include "Equation.h"

extern I2C_HandleTypeDef hi2c1;

MPU6050_dataStruct 			dataStruct;
MPU6050_DATA 						mpu6050;
extern __IO uint32_t 		uwTick;

//-------------------------------------------------------------------Khang----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//user
uint8_t mpu6050_gyro_buf[6];
uint8_t mpu6050_accel_buf[6];
uint8_t mpu6050_magnetic_buf[6];
// define mpu6050 to use in main file
GY86_MPU6050_t MPU6050;

uint8_t  MPU6050_rx;
uint8_t  MPU6050_tx;
float    MPU6050_Gyro_LSB 						= 32.8;
float    MPU6050_Acc_LSB  						= 4096.0;
int 	   samples 											= 0;
int32_t  gyro_x_offset  							= 0;
int32_t  gyro_y_offset  							= 0;
int32_t  gyro_z_offset  							= 0; 
bool  	 isFilterGyro 	 							= false;

// filter
float 	 kalmanAngleRoll  						= 0;
float 	 kalmanUncertaintyAngleRoll 	= 5 * 5;  // uncertainty (std dev) on the initial angle = 2 deg // cao thi loc nhieu tot
float 	 kalmanAnglePitch 						= 0;
float 	 kalmanUncertaintyAnglePitch  = 5 * 5;
float 	 stdDevGyroPitch 							= 10; // 15
float 	 stdDevAccelPitch 						= 20; // 25
float 	 stdDevGyroRoll 						  = 30; // 40
float 	 stdDevAccelRoll 							= 40; // 68


void MPU6050_Start(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu, uint8_t Gyro_FS, uint8_t ACC_FS, uint8_t DLPF_CFG)
{
	if (MPU6050_Init(I2Cx, Gyro_FS, ACC_FS, DLPF_CFG) != 0)
	{
		//fail
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
		osDelay(500);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
		osDelay(500);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
		osDelay(500);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
		osDelay(500);
	};
	isFilterGyro 		= true;
	mpu->FilterCoef = 0.99;
	mpu->PreGyro_X_RAW = 0;
	mpu->PreGyro_Y_RAW = 0;
	mpu->PreGyro_Z_RAW = 0;
//	CalibMPU6050(I2Cx, mpu);

//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
//	osDelay(1000);
}
void HMC5883L_Start(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu)
{
	MPU6050_Bypass(I2Cx);
	HMC5883L_Setup(I2Cx);					
	MPU6050_Master(I2Cx);
	MPU6050_Slave_Read(I2Cx);
//	CalibHMC5883L(I2Cx, mpu);
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
//	osDelay(1000);
}
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx, uint8_t Gyro_FS, uint8_t Acc_FS, uint8_t DLPF_CFG)
{
	//Save LSB/Unit for both gyro and acc in order to use them later
	switch(Gyro_FS)
	{
	case 0: //250dps
		MPU6050_Gyro_LSB = 131.0;
		break;
	case 1: //500dps
		MPU6050_Gyro_LSB = 65.5;
		break;
	case 2: //1000dps
		MPU6050_Gyro_LSB = 32.8;
		break;
	case 3: //2000dps
		MPU6050_Gyro_LSB = 16.4;
		break;
	default:
		break;
	}

	switch(Acc_FS)
	{
	case 0: //2g
		MPU6050_Acc_LSB = 16384.0;
		break;
	case 1: //4g
		MPU6050_Acc_LSB = 8192.0;
		break;
	case 2: //8g
		MPU6050_Acc_LSB = 4096.0;
		break;
	case 3: //16g
		MPU6050_Acc_LSB = 2048.0;
		break;
	default:
		break;
	}

	// Read Who am I
	while(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &MPU6050_rx, 1, 100)!=HAL_OK){};
	MPU6050_tx = 0; //Will return this value if code ends here

	// 0x68 will be returned if sensor accessed correctly and Check device is ready
	if (MPU6050_rx == 0x68 && HAL_I2C_IsDeviceReady(I2Cx, MPU6050_ADDR , 1, 100) == HAL_OK)
	{

		MPU6050_tx = 0;
		while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};

		MPU6050_tx = 19; // Set No Sampling: Sampling Rate = 50 HZ
		while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};

		MPU6050_tx = DLPF_CFG; // Digital Low Pass Filter Setting
		while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, CONFIG_REG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};

		MPU6050_tx = Gyro_FS << 3;
		while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};

		MPU6050_tx = Acc_FS << 3;
		while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};

		return 0;
	}
	return 1;
}
void CalibMPU6050(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu)
{	
		// giu cho mpu6050 nam yen 
		while (samples <8000)
		{
			MPU6050_Read_Gyro_DMA(I2Cx, mpu, 0);
			if(samples<100) 
			{
				samples ++;
			} 
			else
			{
				gyro_x_offset += mpu->Gyro_X_RAW;
				gyro_y_offset += mpu->Gyro_Y_RAW;
				gyro_z_offset += mpu->Gyro_Z_RAW;
				samples ++;
			}
		}
		
		mpu->Gyro_X_Offset_RAW = mpu->Gyro_X_RAW; //(int16_t) (gyro_x_offset / 9900.0);
		mpu->Gyro_Y_Offset_RAW = (int16_t) (gyro_y_offset / 9900.0); //mpu->Gyro_Y_RAW;
		mpu->Gyro_Z_Offset_RAW = mpu->Gyro_Z_RAW; //(int16_t) (gyro_z_offset / 9900.0);
		
		mpu->rollRateOffset 	 = mpu->Gyro_X_Offset_RAW / MPU6050_Gyro_LSB;
		mpu->pitchRateOffset   = mpu->Gyro_Y_Offset_RAW / MPU6050_Gyro_LSB;
		mpu->yawRateOffset     = mpu->Gyro_Z_Offset_RAW / MPU6050_Gyro_LSB;
		
		
		mpu->VerticalVelocity  = 0;
		mpu->isCalibGyro   	   = true;
}

void MPU6050_Read_Gyro_DMA(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu,int isOffset)
{
	while(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, mpu6050_accel_buf, 6,1000)!=HAL_OK){};
//	while(HAL_I2C_Mem_Read_DMA(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, mpu6050_accel_buf, 6)!=HAL_OK){};
//	HAL_I2C_Mem_Read_IT(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, mpu6050_accel_buf, 6);
	mpu->Accel_X_RAW = (int16_t) (mpu6050_accel_buf[0] << 8 | mpu6050_accel_buf[1]);
	mpu->Accel_Y_RAW = (int16_t) (mpu6050_accel_buf[2] << 8 | mpu6050_accel_buf[3]);
	mpu->Accel_Z_RAW = (int16_t) (mpu6050_accel_buf[4] << 8 | mpu6050_accel_buf[5]);
	
	while(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, mpu6050_gyro_buf, 6,1000)!=HAL_OK){};
//	HAL_I2C_Mem_Read_IT(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, mpu6050_gyro_buf, 6);
	mpu->Gyro_X_RAW = (int16_t) (mpu6050_gyro_buf[0] << 8 | mpu6050_gyro_buf[1]);
	mpu->Gyro_Y_RAW = (int16_t) (mpu6050_gyro_buf[2] << 8 | mpu6050_gyro_buf[3]);
	mpu->Gyro_Z_RAW = (int16_t) (mpu6050_gyro_buf[4] << 8 | mpu6050_gyro_buf[5]);
	
	if (isOffset)
	{
		
		mpu->Gyro_X_RAW -= mpu->Gyro_X_Offset_RAW;
		mpu->Gyro_Y_RAW -= mpu->Gyro_Y_Offset_RAW;
		mpu->Gyro_Z_RAW -= mpu->Gyro_Z_Offset_RAW;
//		if (isFilterGyro)
//		{
//			mpu->PreGyro_X_RAW = mpu->Gyro_X_RAW;
//			mpu->PreGyro_Y_RAW = mpu->Gyro_Y_RAW;
//			mpu->PreGyro_Z_RAW = mpu->Gyro_Z_RAW;
//			isFilterGyro       = false;
//		}
		mpu->PreGyro_X_RAW	 = mpu->FilterCoef * mpu->PreGyro_X_RAW + (1 - mpu->FilterCoef) * mpu->Gyro_X_RAW;
		mpu->PreGyro_Y_RAW	 = mpu->FilterCoef * mpu->PreGyro_Y_RAW + (1 - mpu->FilterCoef) * mpu->Gyro_Y_RAW;
		mpu->PreGyro_Z_RAW	 = mpu->FilterCoef * mpu->PreGyro_Z_RAW + (1 - mpu->FilterCoef) * mpu->Gyro_Z_RAW;
		
		mpu-> Gyro_X_RAW 		 = mpu->PreGyro_X_RAW;
		mpu-> Gyro_Y_RAW 		 = mpu->PreGyro_Y_RAW;
		mpu-> Gyro_Z_RAW 		 = mpu->PreGyro_Z_RAW;
		
	}
		
	mpu->Ax 							 = mpu->Accel_X_RAW / MPU6050_Acc_LSB - 0.043f; // - 0.14575 ; 0.0627
	mpu->Ay 							 = mpu->Accel_Y_RAW / MPU6050_Acc_LSB - 0.0155f; // + 0.05371 - 0.06518
	mpu->Az 							 = mpu->Accel_Z_RAW / MPU6050_Acc_LSB - 0.0131f + 0.004f; // - 0.00757 + 0.00146
	
	mpu->rollRate 				 = mpu->Gyro_X_RAW / MPU6050_Gyro_LSB;
	mpu->pitchRate 				 = mpu->Gyro_Y_RAW / MPU6050_Gyro_LSB;
	mpu->yawRate 					 = mpu->Gyro_Z_RAW / MPU6050_Gyro_LSB;
	
//	if (mpu->yawRate<5 && mpu->yawRate > -5)
//	{
//		mpu->yawRate = 0;
//	}
//	if (mpu->rollRate<5 && mpu->rollRate > -5)
//	{
//		mpu->rollRate = 0;
//	}
//	if (mpu->pitchRate<5 && mpu->pitchRate > -5)
//	{
//		mpu->pitchRate = 0;
//	}
}


void MPU6050_Bypass(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_tx = 0; //0b00000000
	while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, USER_CTRL_REG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){}; //Master Disable

	MPU6050_tx = 2; //0b00000010
	while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_PIN_CFG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){}; //Bypass Enable
}

void MPU6050_Master(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_tx = 0; //0x00
	while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_PIN_CFG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){}; //Disable Bypass

	MPU6050_tx = 34; //0b00100010
	while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, USER_CTRL_REG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){}; //Master Enable

	MPU6050_tx = 13; //0b00001101
	while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, I2C_MST_CTRL, 1, &MPU6050_tx, 1, 100)!=HAL_OK){}; //Master Clock to 400kHz

	MPU6050_tx = 0; //0x00
	while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};
}

void HMC5883L_Setup(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_tx = 24; //Fill Slave0 DO  0b00011000 = 75Hz
	while(HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS << 1, HMC5883L_REG_CONFIG_A, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};

	MPU6050_tx = 32; //Fill Slave0 DO  0b00100000
	while(HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS << 1, HMC5883L_REG_CONFIG_B, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};

	MPU6050_tx = 0x00;
	while(HAL_I2C_Mem_Write(I2Cx, HMC5883L_ADDRESS << 1, HMC5883L_REG_MODE, 1, &MPU6050_tx, 1, 100)!=HAL_OK){}; //Mode: Continuous
}

void MPU6050_Slave_Read(I2C_HandleTypeDef *I2Cx)
{
	MPU6050_tx = HMC5883L_ADDRESS | 0x80; //Access Slave into read mode  => Read slave 0 and address of HMC6883L
	while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, I2C_SLV0_ADDR, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};

	MPU6050_tx = HMC5883L_REG_OUT_X_M; //Slave REG for reading to take place
	while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, I2C_SLV0_REG, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};

	MPU6050_tx = 0x80 | 0x06; //Number of data bytes
	while(HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, I2C_SLV0_CTRL, 1, &MPU6050_tx, 1, 100)!=HAL_OK){};
}
void CalibHMC5883L(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu)
	{	
		// Xoay theo 3 huong de callib
		while (samples <50)
		{
			MPU6050_Read_Magnetic(I2Cx, mpu, 0);
			samples ++;
		}	
		MPU6050_Read_Magnetic(I2Cx, mpu, 0);
		mpu->Mag_X_Max = mpu->Mag_X_RAW;
		mpu->Mag_X_Min = mpu->Mag_X_RAW;
		mpu->Mag_Y_Max = mpu->Mag_Y_RAW;
		mpu->Mag_Y_Min = mpu->Mag_Y_RAW;
		mpu->Mag_Z_Max = mpu->Mag_Z_RAW;
		mpu->Mag_Z_Min = mpu->Mag_Z_RAW;
		samples = 0;
		while (samples <5000)
		{
			MPU6050_Read_Magnetic(I2Cx, mpu, 0);
			if(mpu->Mag_X_RAW > mpu->Mag_X_Max) mpu->Mag_X_Max = mpu->Mag_X_RAW;
			if(mpu->Mag_X_RAW < mpu->Mag_X_Min) mpu->Mag_X_Min = mpu->Mag_X_RAW;

			if(mpu->Mag_Y_RAW > mpu->Mag_Y_Max) mpu->Mag_Y_Max = mpu->Mag_Y_RAW;
			if(mpu->Mag_Y_RAW < mpu->Mag_Y_Min) mpu->Mag_Y_Min = mpu->Mag_Y_RAW;

			if(mpu->Mag_Z_RAW > mpu->Mag_Z_Max) mpu->Mag_Z_Max = mpu->Mag_Z_RAW;
			if(mpu->Mag_Z_RAW < mpu->Mag_Z_Min) mpu->Mag_Z_Min = mpu->Mag_Z_RAW;
			samples ++;
		}	
		mpu->Mag_X_Offset = (mpu->Mag_X_Max + mpu->Mag_X_Min) / 2;
		mpu->Mag_Y_Offset = (mpu->Mag_Y_Max + mpu->Mag_Y_Min) / 2;
		mpu->Mag_Z_Offset = (mpu->Mag_Z_Max + mpu->Mag_Z_Min) / 2;
		
		
	}
void MPU6050_Read_Magnetic(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu, int isOffset)
{
	while(HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MAGNETIC_XOUT_H_REG, 1, mpu6050_magnetic_buf, 6,1000)!=HAL_OK){};
//	HAL_I2C_Mem_Read_IT(I2Cx, MPU6050_ADDR, MAGNETIC_XOUT_H_REG, 1, mpu6050_magnetic_buf, 6);
	mpu->Mag_X_RAW  = (int16_t) (mpu6050_magnetic_buf[0] << 8 | mpu6050_magnetic_buf[1]);
	mpu->Mag_Z_RAW  = (int16_t) (mpu6050_magnetic_buf[2] << 8 | mpu6050_magnetic_buf[3]);
	mpu->Mag_Y_RAW = (int16_t) (mpu6050_magnetic_buf[4] << 8 | mpu6050_magnetic_buf[5]);
	
	if (isOffset)
	{
		mpu->Mag_X_RAW -= mpu->Mag_X_Offset; 
		mpu->Mag_Y_RAW -= mpu->Mag_Y_Offset; 
		mpu->Mag_Z_RAW -= mpu->Mag_Z_Offset; 
	}
	mpu->isCalibMag = true;
}
void GetRollPitchYawAngle(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu, float dt)
{
	float Mx, My;
	float pitchAngleMeasure;
	float rollAngleMeasure;
	
	MPU6050_Read_Gyro_DMA(I2Cx, mpu,1);
	pitchAngleMeasure		= -atan2(mpu->Ax, sqrt(pow(mpu->Ay,2) + pow(mpu->Az,2)))*180/(atan(1)*4); // deg
	rollAngleMeasure		= atan2(mpu->Ay, sqrt(pow(mpu->Ax,2) + pow(mpu->Az,2)))*180/(atan(1)*4); // deg
	mpu->PitchAngleRaw  = pitchAngleMeasure;
	mpu->RollAngleRaw 	= rollAngleMeasure;
	
	KalMan_1D(&kalmanAngleRoll, &kalmanUncertaintyAngleRoll, mpu->rollRate, rollAngleMeasure, stdDevGyroRoll, stdDevAccelRoll, dt); // , fabs(mpu6050.rollRateOffset)
	KalMan_1D(&kalmanAnglePitch, &kalmanUncertaintyAnglePitch, mpu->pitchRate, pitchAngleMeasure, stdDevGyroPitch, stdDevAccelPitch, dt); // , fabs(mpu6050.pitchRateOffset)
	
	mpu->RollAngle 			= kalmanAngleRoll;
	mpu->PitchAngle 		= kalmanAnglePitch;
	
	MPU6050_Read_Magnetic(&hi2c1, mpu, 1);
	Mx = mpu->Mag_X_RAW*cos(mpu->PitchAngle*PI/180.0f) + mpu->Mag_Z_RAW*sin(mpu->PitchAngle*PI/180.0f);
	My = mpu->Mag_X_RAW*sin(mpu->RollAngle*PI/180.0f)*sin(mpu->PitchAngle*PI/180.0f) + mpu->Mag_Y_RAW*cos(mpu->RollAngle*PI/180.0f) - mpu->Mag_Z_RAW*sin(mpu->RollAngle*PI/180.0f)*cos(mpu->PitchAngle*PI/180.0f);
	mpu->YawAngleTilte  = -atan2f(My,Mx)*180.0f/PI; // deg
	mpu->YawAngleLevel  = -atan2f(mpu->Mag_Y_RAW,mpu->Mag_X_RAW)*180.0f/PI; //deg 
	if ( mpu->YawAngleTilte < 0)
	{
		mpu->YawAngleTilte = 360 + mpu->YawAngleTilte;
	}
	if ( mpu->YawAngleLevel < 0)
	{
		mpu->YawAngleLevel = 360 + mpu->YawAngleLevel;
	}
	
}

void GetVerticalVelocity(I2C_HandleTypeDef *I2Cx, GY86_MPU6050_t *mpu, float dt)
{
	float accZInertial_Temp;
//	GetRollPitchYawAngle(I2Cx, mpu, dt);
	accZInertial_Temp = -sin(mpu->PitchAngle*PI/180.0f)* mpu->Ax + cos(mpu->PitchAngle*PI/180.0f)*sin(mpu->RollAngle*PI/180.0f)*mpu->Ay
												+ cos(mpu->PitchAngle*PI/180.0f)*cos(mpu->RollAngle*PI/180.0f)*mpu->Az;
	mpu->AccZInertial = (accZInertial_Temp-1.0f)*9.81f; // m/s^2
	// v = v0 + a*dt
	mpu->VerticalVelocity +=  mpu->AccZInertial * dt;
}

void KalMan_1D(float *kalmanState, float *kalmanUncertainty, float kalmanInput, float kalmanMeasurement, float stDevGyro, float stdDevAccel, double dt)
{
	float F = 1; // state transition matrix 
	float G = dt; // control matrix: 
	float Q = dt*dt*stDevGyro*stDevGyro; // process uncertainty (interval^2 * 4^2) dev of rate = 4 deg/s ,stdDevGyro = 4 (default)
	float H = 1; // observation matrix
	float R = stdDevAccel*stdDevAccel; // measurement uncertainty (interval^2 * 3^2) 3: std dev angle(k) = 3 deg (accelerometer) dt*dt*
	// 1. Predict the current state of the system: S = F * S + G * U
  *kalmanState = F*(*kalmanState)+G*kalmanInput; 
	// 2. calculate the uncertainty of the prediction: P = F * p * F.T + Q
  *kalmanUncertainty = F*(*kalmanUncertainty)*F + Q;
	// 3. Calculate the Kalman gain from the uncertainties on the predictions and measurements
  // L = H * P * H.T  + R, R = 3*3
  // K = P * H.T * 1/L
  float kalmanGain = *kalmanUncertainty*H*1/(H*(*kalmanUncertainty)*H+R);
	// 4. Update the predicted state of the system with the measurement of the state through the Kalman gain
  *kalmanState = *kalmanState + kalmanGain*(kalmanMeasurement-H*(*kalmanState));
	// 5. Update the uncertainty of the predicted state:
  *kalmanUncertainty = (1-kalmanGain)*(*kalmanUncertainty);
}
//float P[2][2] = {{0, 0},{0, 0}};
//float S[2][1] = {{0},{0}}
/*
kalmanMeasurement = Attitude from barometer
kalmanState = [Altitude_kalman, velocity_kalman]
kalmanInput = AccelerometerZ_inertial
*/
void KalMan_2D(float kalmanState[2][1], float kalmanUncertainty[2][2], float kalmanInput, float kalmanMeasurement, float dt)
{
	float F[2][2] = {{1, dt},{0, 1}}; // state transition matrix 
	float G[2][1] = {{0.5f*dt*dt},{dt}}; // control matrix: 0.004
	float G_T[1][2];
	transposeMatrix(1, 2, G, G_T);
	/*
	Q = G * G_T * 10^2
	*/
	float Q[2][2];
	float Q_Temp[2][2];
	multiplyMatrices(2, 1, 2, G, G_T, Q);
	multiplyMatrixWithScalar(2, 2, Q, 0.2*0.2, Q_Temp); // Can change value of coefficient Q: currently in use 10 cm/s^2 = 0.1 m/s^2 (sdt dev)
	assignMatrix(2, 2, Q_Temp, Q);
	
	float H[1][2] = {1, 0}; // observation matrix
	float I[2][2] = {{1,0}, {0,1}}; // Identify matrix
	
	// Can change value of coefficient R: currently in use 30 cm = 0.3 m
	float R[1][1] = {0.25*0.25}; // measurement uncertainty (interval^2 * 30^2) 30: std dev (accelerometer) dt*dt*
	/*
	1. Predict the current state of the system: S = F * S + G * U
	S: State vector  S = [Altitude_kalman, velocity_kalman]
	U: Input variable U = AccelZ_inertial	
	*/
	float S_Temp1[2][1];
	float S_Temp2[2][1];
	multiplyMatrices(2, 2, 1, F, kalmanState, S_Temp1);
	multiplyMatrixWithScalar(2,1,G,kalmanInput, S_Temp2);
	addMatrices(2,1,S_Temp1, S_Temp2, kalmanState);
	/*
	2. Calculate uncertainty of the prediction: P = F * P * F_T + Q
	P: Prediction uncertainty vector
	Q: Process uncertainty
	*/
	float P_Temp1[2][2];
	float P_Temp2[2][2];
	float F_T[2][2];
	transposeMatrix(2, 2, F, F_T);
	multiplyMatrices(2, 2, 2, F, kalmanUncertainty, P_Temp1);
	multiplyMatrices(2, 2, 2, P_Temp1, F_T, kalmanUncertainty);
	addMatrices(2, 2, kalmanUncertainty, Q, P_Temp2);
	assignMatrix(2, 2, P_Temp2, kalmanUncertainty);
	
	/* 
	3. Calculate the Kalman gain from the uncertainty on the prediction and measurement
	L = H * P * H.T  + R 
  K = P * H.T * 1/L 				[2,2] * [2,1] * [1][1]
	*/
	float L[1][1];
	float L_Temp[1][1];
	float H_T[2][1];
	float H_Temp[1][2];
	float KalmanGain[2][1];
	float KalmanGain_Temp[2][1];
	transposeMatrix(1, 2, H, H_T);
	multiplyMatrices(1, 2, 2, H, kalmanUncertainty, H_Temp);
	multiplyMatrices(1, 2, 1, H_Temp, H_T, L_Temp);
	addMatrices(1, 1, L_Temp, R, L);
	
	multiplyMatrices(2, 2, 1, kalmanUncertainty, H_T, KalmanGain_Temp);
	multiplyMatrixWithScalar(2,1,KalmanGain_Temp, 1.0f/L[0][0], KalmanGain);
	
	/*
	4. Update the predicted state of the system with the measurement of the state through the Kalman gain
	S = S + K * (M-H*S) 
	*/
	float scalar_Temp1;
	float scalar_Temp2[1][1];
	float S_Temp3[2][1];
	float KalmanGain_Temp2[2][1];
	multiplyMatrices(1, 2, 1, H, kalmanState, scalar_Temp2);
	scalar_Temp1 = kalmanMeasurement - scalar_Temp2[0][0];
	multiplyMatrixWithScalar(2,1,KalmanGain, scalar_Temp1, KalmanGain_Temp2);
	assignMatrix(2, 1, kalmanState, S_Temp3);
	addMatrices(2, 1, S_Temp3, KalmanGain_Temp2, kalmanState);
	
	/*
	5. Update the uncertainty of the predicted state:
	P = (I - K * H)*P [2][2] - [2][1]*[1][2]
	*/
	float KalmanGain_Temp3[2][2];
	float I_Temp[2][2];
	float P_Temp3[2][2];
	multiplyMatrices(2, 1, 2, KalmanGain, H, KalmanGain_Temp3);
	subtractMatrices(2, 2, I, KalmanGain_Temp3, I_Temp);
	assignMatrix(2, 2, kalmanUncertainty, P_Temp3);
	multiplyMatrices(2, 2, 2, I_Temp, P_Temp3, kalmanUncertainty);
}
