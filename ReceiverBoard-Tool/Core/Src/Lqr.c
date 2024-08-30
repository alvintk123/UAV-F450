#include "Lqr.h"
 
void LQRController(GY86_MPU6050_t *mpu, MS5611_t *MS5611, verticalState	*vertState, float preIntegErrState[4][1], float prevErrState[4], float errState[4], float U[4][1], float dt)
{
	float K[4][8] = {{-265.2976, -51.9142, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
										{0.0, 0.0, 2.5911, 0.0, 0.0, 0.3504, 0.0, 0.0},
										{0.0, 0.0, 0.0, 2.6006, 0.0, 0.0, 0.3567, 0.0},
										{0.0, 0.0, 0.0, 0.0, 2.1998, 0.0, 0.0, 0.3754}};
	float Ki[4][4] = {{-100, 0.0, 0.0, 0.0},
										{0.0, 3.873, 0.0, 0.0},
										{0.0, 0.0, 3.873, 0.0},
										{0.0, 0.0, 0.0, 2.4495}};
	float state[8][1] = {{vertState->altitude},
											 {vertState->velocity},
											 {mpu->RollAngle},
											 {mpu->PitchAngle},
											 {mpu->YawAngleTilte},
											 {mpu->rollRate},
											 {mpu->pitchRate},
											 {mpu->yawRate}};
	float u1[4][1];
	float u2[4][1];
	for (int i = 0; i < 4; i++)
	{
		preIntegErrState[i][0] = preIntegErrState[i][0] + (prevErrState[i] + errState[i])*dt/2;
		prevErrState[i] = errState[i];
	}
	// u1 (4x1) = Ki (4x4) * preIntegErrState (4x1) 
	multiplyMatrices(4, 4, 1, Ki, preIntegErrState, u1);
	// u2 (4x1) = K (4x8) * state (8x1) 
	multiplyMatrices(4, 8, 1, K, state, u2);
	addMatrices(4, 1, u1, u2, U);
	
}
