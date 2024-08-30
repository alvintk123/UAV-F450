#include "user_ReceiverCommunication_thread.h"

extern uint16_t											g_ID_UART_send_u16;
// Data Receiving from controller tool
extern uint8_t											getConnectVehicle;

extern APPLICATION_ADC_INFO					app_adc_infor;
extern APPLICATION_SLAVE_INFO				app_slave_infor;

void ReceiverCommunication_Processing(void)
{
	// ---------------------------------------------------------------------------------------------
		// Process comunication between transmiter board (this board) with receiver board through uart3
//		if (getConnectVehicle)
//		{
			// proccess data receive
			if(g_startProcessData_u8 == PROTOCOL_TRUE)
			{
				// proccess data
				g_startProcessData_u8 = PROTOCOL_FALSE;
				Protocol_process_data_input();
			}
			else{}
				
			// process data send as master board
			#ifdef MASTER_BOARD
			if(app_adc_infor.status == PROTOCOL_TRUE)
			{
				Protocol_UART_send_respond_package(CMD_TRANSMIT_ADC_CONTROL, g_ID_UART_send_u16++, RESPOND_SUCESS);	
			}
			else
			{
			}
			#else
			#endif	
//		}
			osDelay(100);
}
