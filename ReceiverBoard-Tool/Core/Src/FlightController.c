
#include "FlightController.h"
#include "main.h"

extern MS5611_t 										ms5611_1;
extern MS5611_t 										ms5611_2;
extern GY86_MPU6050_t 							MPU6050;
extern TIM_HandleTypeDef 						htim4;
extern APPLICATION_SLAVE_INFO		  	app_slave_infor;
extern APPLICATION_SLAVE_PID				app_slave_pid;
extern APPLICATION_ADC_INFO					app_adc_infor;

// Thrust and Torque Coefficient 
float KT;  
float KM;
float Lx = 0.16;
float Ly = 0.159;
float maxOm = 12000;
float minOm = 2800;
float U[4][1];

// type of controller 
int controller = 1; // 1: PID, 2: LQR

// LQR Coef
float preIntegErrState[4][1] = {{0},{0},{0},{0}};
float prevErrState[4] = {0, 0, 0, 0};
float errState[4];

// Command
float InputThrottle;
float InputRoll 				= 0;
float InputPitch 				= 0;
float InputYaw 					= 0;

float ThrottleCutOff 		= 550;

// Variable for rate PID
float inputThrust;
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

// PID Constants for rate (rad/s)
float PRateRoll=0.0026723788806122;   	// yaw 0.09 pitch = roll =0.006
float IRateRoll=0.000328842980010342;  	//yaw 0.002 pitch = roll = 0.035
float DRateRoll=0.00230994974838175;  	// yaw 0 pitch = roll = 0.0003 

float PRatePitch=0.0192435242957082;
float IRatePitch=0.0105443651965256;
float DRatePitch=0.00181213048672737;

float PRateYaw=0.0101845503734512;
float IRateYaw=0.00206541263124236;
float DRateYaw=0.00168458436200219;

// %throttle for each motor
float MotorInput[4];
// Variable for Angle PID
float DesiredAngleRoll,					 DesiredAnglePitch;
float ErrorAngleRoll, 					 ErrorAnglePitch;
float PrevErrorAngleRoll, 			 PrevErrorAnglePitch;
float PrevItermAngleRoll, 			 PrevItermAnglePitch;
float DesiredVerticalVelocity, 	 ErrorVerticalVelocity;
float PrevErrorVerticalVelocity, PrevItermVerticalVelocity;

// PID Constants for Roll, Pitch angle (rad) and Vertical Velocity (m/s)
float PAngleRoll=0.957136999118894f; 
float IAngleRoll=0.120955041924529f; 
float DAngleRoll=1.31056431289667; 

float PAnglePitch=0.733600878433523;
float IAnglePitch=0.0810740522540832;
float DAnglePitch=0.77541808029016;
float PVerticalVelocity = 3.35f;		// 3.5
float IVerticalVelocity = 0.005f; // 0.4
float DVerticalVelocity = 0.003f;

/*
				Coordinate: X points forward
										Y points to left
										Z points up
				Motor 1: In right front (CCW)
				Motor 2: in right back (CW)
				Motor 3: In left back (CCW)
				Motor 4: In left front (CW)
				
				-> Method: Output of each motor is a combination of the Throttle, Roll, Pitch, Yaw
*/
				
void ControlLoop(controlMotor* control, verticalState* vertState, float dt, bool checkStartMotor)
{
	
	KT = pow(10.0,-7.0);
	KM = 9*pow(10.0,-10.0);
	float K_UToOm[4][4] = {{1/(4*KT), -1/(4*KT*Lx), 1/(4*KT*Ly), 1/(4*KM)},
												{1/(4*KT), -1/(4*KT*Lx), -1/(4*KT*Ly), -1/(4*KM)},
												{1/(4*KT), 1/(4*KT*Lx), -1/(4*KT*Ly), 1/(4*KM)},
												{1/(4*KT), 1/(4*KT*Lx), 1/(4*KT*Ly), -1/(4*KM)}};
	
	
	/* Read input control from joysitck (ADC signal)
	Range Adc: 1600 -> 2600 (Assumed)
	
	Desired RateRoll: -75 -> 75 (deg/s) 
	Desired RatePitch: -75 -> 75 (deg/s) 
	Desired RateYaw: -75 -> 75 (deg/s) 
	Desired Roll: -40 -> 40 (deg)
	Desired Pitch: -40 -> 40 (deg)
	Desired Vertical Velocity: -2 -> 2 (m/s)
	
	%Throttle: 550 -> 970 
	*/
	if (app_adc_infor.isTuneThrottle)
	{
//		ThrottleHover							 		= app_adc_infor.throttleHover;
//		app_slave_infor.throttleHover = ThrottleHover;
	}
	if (app_adc_infor.isTunePID)	
	{	
		
		
		PVerticalVelocity 				 = app_adc_infor.PtermV;
		IVerticalVelocity 				 = app_adc_infor.ItermV;
		DVerticalVelocity 				 = app_adc_infor.DtermV;

		PRateYaw 									 = app_adc_infor.PtermY;
		IRateYaw 									 = app_adc_infor.ItermY;
		DRateYaw 									 = app_adc_infor.DtermY;
		
		PRatePitch 								 = app_adc_infor.PtermP;
		IRatePitch 								 = app_adc_infor.ItermP;
		DRatePitch 								 = app_adc_infor.DtermP;
		
		PRateRoll 								 = app_adc_infor.PtermR;
		IRateRoll 								 = app_adc_infor.ItermR;
		DRateRoll 								 = app_adc_infor.DtermR;
		
		
//		app_slave_infor.PTermCoefV = PVerticalVelocity;
//		app_slave_infor.ITermCoefV = IVerticalVelocity;
//		app_slave_infor.DTermCoefV = DVerticalVelocity;
//		
//		app_slave_infor.PTermCoefY = PRateYaw;
//		app_slave_infor.ITermCoefY = IRateYaw;
//		app_slave_infor.DTermCoefY = DRateYaw;
//		
//		app_slave_infor.PTermCoefP = PRatePitch;
//		app_slave_infor.ITermCoefP = IRatePitch;
//		app_slave_infor.DTermCoefP = DRatePitch;
//		
//		app_slave_infor.PTermCoefR = PRateRoll;
//		app_slave_infor.ITermCoefR = IRateRoll;
//		app_slave_infor.DTermCoefR = DRateRoll;
	}
	if (checkStartMotor)
	{
		switch (controller)
		{
			case 1:
				DesiredVerticalVelocity = control->verticalVelocityControl;
				DesiredRateYaw 					= control->yawControl*PI/180.0f;
				DesiredAnglePitch				= control->pitchControl*PI/180.0f;
				DesiredAngleRoll 				= control->rollControl*PI/180.0f;
				
				ErrorVerticalVelocity   = DesiredVerticalVelocity - vertState->velocity;
			
//				app_slave_infor.DTerm 										= (float) (DVerticalVelocity *(ErrorVerticalVelocity-PrevErrorVerticalVelocity)/(dt)); // 
				
				// Vertical Velocity PID
				PID_Equation(ErrorVerticalVelocity, PVerticalVelocity, IVerticalVelocity, DVerticalVelocity, &PrevErrorVerticalVelocity, &PrevItermVerticalVelocity, dt, &inputThrust, 'T');
				InputThrottle 														= inputThrust; // value when joystick in middle
				
//				app_slave_infor.ErrorVerticalVelocity 		= ErrorVerticalVelocity;
//				app_slave_infor.PrevItermVerticalVelocity = PrevItermVerticalVelocity; // PrevItermVerticalVelocity
//				app_slave_infor.inputThrottleTemp 				= inputThrust;
//				app_slave_infor.PTerm 										= PVerticalVelocity  * ErrorVerticalVelocity; // * ErrorVerticalVelocity
				
				// Outer Loop: Angle PID

				// Calculate Error between Measured value and Desired value (PID tuned for angle in rad)
				ErrorAngleRoll = DesiredAngleRoll - MPU6050.RollAngle*PI/180.0f;
				ErrorAnglePitch = DesiredAnglePitch - MPU6050.PitchAngle*PI/180.0f;
				
				
				// Apply Angle PID -> output: input for iner loop 
				PID_Equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, &PrevErrorAngleRoll, &PrevItermAngleRoll, dt, &DesiredRateRoll, 'R');
				PID_Equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, &PrevErrorAnglePitch, &PrevItermAnglePitch, dt, &DesiredRatePitch, 'P');
				
				
				// DesiredRateRoll and DesiredRatePitch is input for iner loop
				ErrorRateRoll		= DesiredRateRoll-MPU6050.rollRate;
				ErrorRatePitch	= DesiredRatePitch-MPU6050.pitchRate;
				ErrorRateYaw		= DesiredRateYaw-(-MPU6050.yawRate*PI/180.0f);
				
				// Iner Loop: Rate PID
					// Apply Rate PID -> output: control speed of motor
				
				PID_Equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, &PrevErrorRateRoll, &PrevItermRateRoll, dt, &InputRoll, 'L');
				PID_Equation(ErrorRatePitch, PRatePitch,IRatePitch, DRatePitch, &PrevErrorRatePitch, &PrevItermRatePitch, dt, &InputPitch, 'M');
				PID_Equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, &PrevErrorRateYaw, &PrevItermRateYaw, dt, &InputYaw, 'N');
				
				U[0][0] = InputThrottle;
				U[1][0] = InputRoll;
				U[2][0] = InputPitch;
				U[3][0] = InputYaw;
				
				//  
//				app_slave_infor.ErrorYawRate 		 = ErrorRateYaw;
//				app_slave_infor.PrevItermYawRate = PrevItermRateYaw;
//				app_slave_infor.PTermYawRate     = PRateYaw*ErrorRateYaw;
//				app_slave_infor.DTermYawRate		 = DRateYaw*(ErrorRateYaw-PrevErrorRateYaw)/dt;
//				app_slave_infor.inputYawTemp 		 = InputYaw;
				// Remaining %throttle use for control roll, pitch ,yaw for backup
				break;
			
			case 2:
				errState[0] = control->verticalVelocityControl - vertState->altitude;
				errState[1] = control->rollControl - MPU6050.RollAngle;
				errState[2] = control->pitchControl - MPU6050.PitchAngle;
				errState[3] = control->yawControl - MPU6050.YawAngleTilte;
			
//				LQRController(&MPU6050, &ms5611_1, vertState, preIntegErrState, prevErrState, errState, U, dt);
				break;
					
		}

		/*
		Todo: Control motor
		Note: Connect ESC to hardware according order
		*/
		// Convert from Thrust and Torque to Speed of Motor and Restrict the thrust from increasing too high or decreasing too low
		
		float omSquare[4][1];
		float omTemp;
		float om[4];
		multiplyMatrices(4, 4, 1, K_UToOm, U, omSquare);
		
		for (int i = 0; i < 4; i++)
		{
			if (omSquare[i][0] < 0)
			{
				omTemp = -sqrt(-omSquare[i][0]);
			}
			else
			{
				omTemp = sqrt(omSquare[i][0]);
			}
			if (omTemp >= maxOm)
			{
				omTemp = maxOm;
			}
			else if (omTemp <= minOm)
			{
				omTemp = minOm;
			}
			om[i] = omTemp;
		}
		
		// Convert from speed of motor to input throttle
		for (int i = 0; i < 4; i++)
		{
			MotorInput[i] = 0.0000033302f*pow(om[i],2)-0.0098361388f*om[i]+605.4058310219f;
		}
		
		runMotor(&htim4, MotorInput);
	}
	else
	{
		if (controller == 1)
		{
			PID_Reset(&PrevErrorRateRoll, &PrevErrorRatePitch, &PrevErrorRateYaw, &PrevItermRateRoll, &PrevItermRatePitch, 
							&PrevItermRateYaw, &PrevErrorAngleRoll, &PrevErrorAnglePitch, &PrevItermAngleRoll, &PrevItermAnglePitch,
							&PrevErrorVerticalVelocity, &PrevItermVerticalVelocity);
		}
		for (int i = 0; i < 4; i++)
		{
			MotorInput[i] = ThrottleCutOff;
		}
	
		runMotor(&htim4, MotorInput);
	}
//	app_slave_infor.inputThrottle1 = MotorInput[0];
//	app_slave_infor.inputThrottle2 = MotorInput[1];
//	app_slave_infor.inputThrottle3 = MotorInput[2];
//	app_slave_infor.inputThrottle4 = MotorInput[3];

}
