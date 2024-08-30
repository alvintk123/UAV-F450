#include "user_ToolCommunication_thread.h"

extern uint8_t 			rxm[40];


//// Data for Sending
//extern uint8_t 			isConnectVehicleFlag;
//extern uint8_t   		isCalibSensorFlag;
//extern uint8_t   		isStartSensorFlag;
//extern uint8_t   		isCalibMotorFlag;
//extern uint8_t   		isOnMotorFlag;
//extern uint8_t   		isStateResponseFlag;
//extern uint8_t   		isPlot3DAnimationFlag;
//extern uint8_t   		isPlotRefResFlag;

//// Data Receiving from controller tool
//extern uint8_t				getConnectVehicle;
//extern uint8_t				startCalibSensor;
//extern uint8_t				startSensor;
//extern uint8_t				startCalibMotor;
//extern uint8_t				startMotor;
//extern uint8_t				getStateResponse;
//extern uint8_t				plot3DAnimation;
//extern uint8_t				plotRefRes;

//extern APPLICATION_ADC_INFO					app_adc_infor;
//extern APPLICATION_SLAVE_INFO				app_slave_infor;

extern char 								feedbackFlagData[17]; 
extern char 								feedbackFlagStateData[83]; 
extern bool 								isSendFeedbackFlag;
extern bool									isSendFeedbackFlagState;
extern bool 								isSendData;
//extern double motorSpeed1;
//extern double motorSpeed2;
//extern double motorSpeed3;
//extern double motorSpeed4;

extern UART_HandleTypeDef 	huart2;
void ToolCommunication_Initization(void)
{
	HAL_UART_Receive_DMA(&huart2, rxm, sizeof(rxm));
	osDelay(5);
	
}
void ToolCommunication_Processing(void)
{	
	if (isSendFeedbackFlag && isSendFeedbackFlagState && isSendData)
	{
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*) feedbackFlagStateData, sizeof(feedbackFlagStateData));
//		HAL_UART_Transmit_IT(&huart2 ,(uint8_t*) feedbackFlagStateData, sizeof(feedbackFlagStateData));
		isSendData = false;
	}
	else if (isSendFeedbackFlag && isSendData) // 
	{
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*) feedbackFlagData, sizeof(feedbackFlagData));
//		HAL_UART_Transmit_IT(&huart2 ,(uint8_t*) feedbackFlagData, sizeof(feedbackFlagData));
		isSendData = false;
	}
	osDelay(50);
}

//void ToolCommunication_Processing(void)
//{	
////	if (!isSendData)
////	{
//		HAL_UART_Receive_IT(&huart2, rxm, sizeof(rxm));
//		isSendData = true;
////	}
//	osDelay(50);
//}

