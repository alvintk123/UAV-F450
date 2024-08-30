
#include "user_Infor_thread.h"
#include "main.h"
#include "stdio.h"
#include "protocol.h"

extern MS5611_t 								ms5611_1;
extern GY86_MPU6050_t 					MPU6050;
extern verticalState 						vertState;
//extern UART_HandleTypeDef				huart2;
extern controlMotor 						Motor;

//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif

//PUTCHAR_PROTOTYPE
//{
//	while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX) {};
//  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1);
//	
//  return ch;
//}

uint16_t g_ADC_DMA_u16[2];
APPLICATION_FLASH_INFO 				app_info;
APPLICATION_ADC_INFO					app_adc_infor;
APPLICATION_SLAVE_INFO		  	app_slave_infor;
APPLICATION_SLAVE_PID					app_slave_pid;

void Infor_Initization(void)
{
	Protocol_Initial();
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)g_ADC_DMA_u16, 2);
}

void Infor_Processing(void)
{
		osDelay(10);
	// remove to alway communication -- 
//		if (checkCalibSensorBeforeStartMotor(&MPU6050, &ms5611_1, &vertState))
//		{
//			
//			if (HAL_GPIO_ReadPin(Xbee_GPIO_Port, Xbee_Pin) != GPIO_PIN_SET)
//			{
//				HAL_GPIO_WritePin(Xbee_GPIO_Port, Xbee_Pin, 1);
//			}
//		}
		if(g_startProcessData_u8 == PROTOCOL_TRUE)
		{
			// proccess data
			g_startProcessData_u8 = 0;
			Protocol_process_data_input();
		}
		else{}
}

void system_resetMCU(void){
    NVIC_SystemReset();
    //USBD_Reset(0);
    osDelay(2800);
    while(1);
}

void Infor_SetBuzzerBeep(uint8_t timeBeep, uint16_t timeDelay)
{
	do
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		osDelay(timeDelay);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		osDelay(timeDelay);
	} while(--timeBeep);
}
