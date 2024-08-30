//#include "main.h"
#include "string.h"
#include "joystick.h"
#include "protocol.h"

// declare variable
drone_get_cmd_e 					get_cmd_stt;
drone_cmd_t cmd_send, cmd_get, cmd_process;
PAYLOAD_DRONE_INFOR 		  drone_infor;

uint32_t g_tickstart_u32 = 0, g_ticksByte_u32 = 0;
uint8_t g_rxBuf_u8 = 0;
uint16_t getIndex = 0;
uint8_t g_startProcessData_u8 = PROTOCOL_FALSE;
char data_sent[UART_PACK_SIZE_MAX];
// Uart 2
extern char 													feedbackFlagData[17];
extern char 													feedbackFlagStateData[83];
extern bool 													isSendFeedbackFlag;
extern bool														isSendFeedbackFlagState;
extern bool														isSendMessage3;
extern bool 													isSendData;

extern uint8_t 												isConnectVehicleFlag;
extern uint8_t   											isCalibSensorFlag;
extern uint8_t   											isStartSensorFlag;
extern uint8_t   											isCalibMotorFlag;
extern uint8_t   											isOnMotorFlag;
extern uint8_t   											isStateResponseFlag;
extern uint8_t   											isPlot3DAnimationFlag;
extern uint8_t   											isPlotRefResFlag;

extern double 												motorSpeed1;
extern double 												motorSpeed2;
extern double 												motorSpeed3;
extern double 												motorSpeed4;

// Data Receiving from controller tool
extern uint8_t 												rxm[40];

extern uint8_t												getConnectVehicle;
extern uint8_t												startCalibSensor;
extern uint8_t												startSensor;
extern uint8_t												startCalibMotor;
extern uint8_t												startMotor;
extern uint8_t												getStateResponse;
extern uint8_t												plot3DAnimation;
extern uint8_t												plotRefRes;
extern joystickCommandProperties 			joystickCmd;
//
APPLICATION_FLASH_INFO 								app_info;
APPLICATION_ADC_INFO									app_adc_infor;
APPLICATION_SLAVE_INFO		  					app_slave_infor;
//extern bool 													isSaveCalibData;
//extern bool 													isLoadCalibData;
//extern bool 													isSaveHMC5883 ;
//extern bool 													isLoadHMC5883 ;



void Protocol_Initial(void)
{
	// turn on Xbee reset Xbee
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(Xbee_Master_GPIO_Port, Xbee_Master_Pin, GPIO_PIN_SET);
	// init value ADC
	app_adc_infor.status = PROTOCOL_TRUE;
	app_adc_infor.pitchControl 			= 0;
	app_adc_infor.rollControl 			= 0;
	app_adc_infor.thrustControl 		= 0;
	app_adc_infor.yawControl 				= 0;
	app_adc_infor.PtermV						= 3.35;
	app_adc_infor.ItermV						= 0.005;
	app_adc_infor.DtermV						= 0.003;
	app_adc_infor.PtermY						= 0.9;
	app_adc_infor.ItermY						= 1.2;
	app_adc_infor.DtermY						= 0;
	app_adc_infor.PtermP						= 0; // 0.006
	app_adc_infor.ItermP						= 0; // 0.035
	app_adc_infor.DtermP						= 0; // 0.0003
	app_adc_infor.PtermR						= 0;
	app_adc_infor.ItermR						= 0;
	app_adc_infor.DtermR						= 0;
	app_adc_infor.isTunePID 				= false;
	app_adc_infor.isTuneThrottle		= false;
	
	app_adc_infor.startCalibMotor		= false;
	app_adc_infor.startMotor 				= false;
	app_adc_infor.startCalibSensor  = false;
	app_adc_infor.startSensor  			= false;
	
	// if code is loop in Bootloader => we return all information is 00
	strncpy(drone_infor.fw_version, "2.00", 4);
  strncpy(drone_infor.hw_version, "2.00", 4);
  strncpy(drone_infor.Serial_number, "123456789012", 12);
	// change define UART using for project
	HAL_UART_Receive_IT(&huart3, &g_rxBuf_u8, 1);
	
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if (huart->Instance == USART2)
	{
		// ---------------------------------------------------------------------------------
		if (isSendFeedbackFlag && isSendFeedbackFlagState)
		{			
			sprintf(feedbackFlagStateData, "%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%+2.1f,%+2.1f,%+2.1f,%+2.1f,%+2.1f,%+5.1f,%+5.1f,%+5.1f,%+5.1f\r\n"
				,isConnectVehicleFlag, isCalibSensorFlag, isStartSensorFlag, isCalibMotorFlag, isOnMotorFlag, isStateResponseFlag, isPlot3DAnimationFlag,
				isPlotRefResFlag, app_slave_infor.RollAngle, app_slave_infor.PitchAngle, app_slave_infor.YawAngleLevel, app_slave_infor.realVel,
				app_slave_infor.YawRate, motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4);
		}
		else if (isSendFeedbackFlag)
		{
			sprintf(feedbackFlagData, "%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d\r\n" ,isConnectVehicleFlag,
				isCalibSensorFlag, isStartSensorFlag, isCalibMotorFlag, isOnMotorFlag, isStateResponseFlag,
				isPlot3DAnimationFlag, isPlotRefResFlag);
		}
//		HAL_UART_Receive_DMA(&huart2, rxm, sizeof(rxm));
//		HAL_UART_Receive_IT(&huart2, rxm, sizeof(rxm));
		//-------------------------------------------------------------------------------------
//		isSendData = true;
	}
		
	
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    uint8_t l_temp_u8 = 0, *p_u8;
    uint32_t l_ticks_u32 = HAL_GetTick();
// change define UART using for project
	
		if (huart->Instance == USART2)
		{

			// -----------------------------------------------------------------------------------------------
			getConnectVehicle 	= rxm[0];
			startCalibSensor		= rxm[1];
			startSensor 				= rxm[2];
			startCalibMotor 		= rxm[3];
			startMotor 					= rxm[4];
			getStateResponse 		= rxm[5];
			plot3DAnimation 		= rxm[6];
			plotRefRes 					= rxm[7];
			
			uint8_t throttleData[] = {rxm[8], rxm[9], rxm[10], rxm[11], rxm[12], rxm[13], rxm[14], rxm[15]};
			uint8_t yawData[] 		 = {rxm[16], rxm[17], rxm[18], rxm[19], rxm[20], rxm[21], rxm[22], rxm[23]};
			uint8_t rollData[] 		 = {rxm[24], rxm[25], rxm[26], rxm[27], rxm[28], rxm[29], rxm[30], rxm[31]};
			uint8_t pitchData[] 	 = {rxm[32], rxm[33], rxm[34], rxm[35], rxm[36], rxm[37], rxm[38], rxm[39]};
			
			joystickCmd.throttleCommand 	= *(double*)&throttleData;
			joystickCmd.yawCommandDeg 		= *(double*)&yawData;
			joystickCmd.rollCommandDeg 		= *(double*)&rollData;
			joystickCmd.pitchCommandDeg 	= *(double*)&pitchData;
			isSendData 										= true;
			//-----------------------------------------------------------------------------------------------------
//			if (isSendFeedbackFlag && isSendFeedbackFlagState)
//			{			
//				sprintf(feedbackFlagStateData, "%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%+2.1f,%+2.1f,%+2.1f,%+2.1f,%+2.1f,%+5.1f,%+5.1f,%+5.1f,%+5.1f\r\n"
//					,isConnectVehicleFlag, isCalibSensorFlag, isStartSensorFlag, isCalibMotorFlag, isOnMotorFlag, isStateResponseFlag, isPlot3DAnimationFlag,
//					isPlotRefResFlag, app_slave_infor.RollAngle, app_slave_infor.PitchAngle, app_slave_infor.YawAngleLevel, app_slave_infor.realVel,
//					app_slave_infor.YawRate, motorSpeed1, motorSpeed2, motorSpeed3, motorSpeed4);
//				HAL_UART_Transmit_IT(&huart2 ,(uint8_t*) feedbackFlagStateData, sizeof(feedbackFlagStateData));
//			}
//			else if (isSendFeedbackFlag)
//			{
//				sprintf(feedbackFlagData, "%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d\r\n" ,isConnectVehicleFlag,
//					isCalibSensorFlag, isStartSensorFlag, isCalibMotorFlag, isOnMotorFlag, isStateResponseFlag,
//					isPlot3DAnimationFlag, isPlotRefResFlag);
//				HAL_UART_Transmit_IT(&huart2 ,(uint8_t*) feedbackFlagData, sizeof(feedbackFlagData));
//					
//			}
			
		}
	
		if(huart->Instance == USART3)
		{
				l_temp_u8 = g_rxBuf_u8;
				if(l_ticks_u32 - g_ticksByte_u32 > TIMEOUT_GET_BYTE)
				{
					get_cmd_stt = DRONE_GET_HEADER0;
          g_ticksByte_u32 = l_ticks_u32;
				}
				switch((uint8_t)get_cmd_stt)
				{
					case DRONE_GET_HEADER0:
					{
						if(l_temp_u8 == HEADER0)
						{
							get_cmd_stt = DRONE_GET_HEADER1;						
						}
						else{}
					   break;
					}
					case DRONE_GET_HEADER1:
					{
						if(l_temp_u8 == HEADER1)
						{
							get_cmd_stt = DRONE_GET_LEN0;
              cmd_get.header = HEADER_16;
						}
						else if(l_temp_u8 != HEADER1)
						{
							get_cmd_stt = DRONE_GET_HEADER0;
						}
						else{}
					   break;
					}
					case DRONE_GET_LEN0:
					{
						get_cmd_stt = DRONE_GET_LEN1;
            cmd_get.len = l_temp_u8;
            getIndex = 0;
					   break;
					}
					case DRONE_GET_LEN1:
					{
						get_cmd_stt = DRONE_GET_TYPE;
            cmd_get.len += (uint16_t)l_temp_u8 << 8;
            getIndex++;
					   break;
					}
					case DRONE_GET_TYPE:
					{
						get_cmd_stt = DRONE_GET_TRAN_ID0;
            cmd_get.type = l_temp_u8;
            getIndex++;
					   break;
					}
					case DRONE_GET_TRAN_ID0:
					{
						get_cmd_stt = DRONE_GET_TRAN_ID1;
            cmd_get.id = l_temp_u8;
            getIndex++;
					   break;
					}
					case DRONE_GET_TRAN_ID1:
					{
						get_cmd_stt = DRONE_GET_OTHER;
            cmd_get.id += (uint16_t)l_temp_u8 << 8;
            getIndex++;
					   break;
					}
					case DRONE_GET_OTHER:
					{
						p_u8 = (uint8_t*)&cmd_get.len + 1;
            p_u8 += getIndex;
            *p_u8 = l_temp_u8;

            if(++getIndex >=cmd_get.len){
							 // copy data when finish array
                memcpy((void*)&cmd_process, (void*)&cmd_get, cmd_get.len + 3);
                get_cmd_stt = DRONE_GET_HEADER0;
                g_startProcessData_u8 = PROTOCOL_TRUE;
            }
            else{
							// repeat when finish data
                get_cmd_stt = DRONE_GET_OTHER;
            }
					   break;
					}			
				}
			HAL_UART_Receive_IT(huart, &g_rxBuf_u8, 1);
		}

}
void Protocol_send_response_data(char *str_, uint16_t len)
{
    HAL_UART_Transmit_IT(&huart3, (uint8_t *)str_, len);
}
unsigned char Protocol_UART_send_respond_package(unsigned char cmd, uint16_t ID, unsigned char status)
{
    unsigned char ck;
    if(status == RESPOND_CHECKSUM_ERROR){
        cmd_send.payload[0] = RESPOND_FAILED_CHECKSUM;
        cmd_send.len = 7;
    }
    else
    {
        switch(cmd){
        case CMD_READ_VERSION:
            drone_infor.Status = RESPOND_SUCESS;
            memcpy(cmd_send.payload, &drone_infor, sizeof(drone_infor));
            cmd_send.len = 6 + sizeof(drone_infor);
            break;
				case CMD_TRANSMIT_ADC_CONTROL:
						// if master board => send data to slave
						// if slave board => send feedback with roll, pitch, temp, high, status
						#ifdef MASTER_BOARD
							app_adc_infor.thrustControl 	= joystickCmd.throttleCommand;
							app_adc_infor.yawControl 			= joystickCmd.yawCommandDeg;
							app_adc_infor.pitchControl 		= joystickCmd.pitchCommandDeg; // g_ADC_DMA_u16
							app_adc_infor.rollControl 		= joystickCmd.rollCommandDeg;
							app_adc_infor.isOnMotor 			= isOnMotorFlag;
							app_adc_infor.status 					= PROTOCOL_TRUE;
							memcpy(cmd_send.payload, &app_adc_infor, sizeof(app_adc_infor));
							cmd_send.len = 6 + sizeof(app_adc_infor);
						#else
							app_slave_infor.rollValue = 0;
							app_slave_infor.pitchValue = 0;
							app_slave_infor.tempValue = 0;
							app_slave_infor.highValue = 0;
							app_slave_infor.status = PROTOCOL_TRUE;
							memcpy(cmd_send.payload, &app_slave_infor, sizeof(app_slave_infor));
							cmd_send.len = 6 + sizeof(app_slave_infor);
						#endif
					break;
        case CMD_JUMP_BOOTLOADER:
        default:
            cmd_send.payload[0] = status;
            cmd_send.len = 7;
            break;
        }
    }
    cmd_send.header = 0xFF55;
    cmd_send.type  = RESPONSE_PACKAGE;
    cmd_send.id = cmd_process.id;
    cmd_send.opcode = cmd;
    if(cmd_send.len > UART_PACK_SIZE_MAX) return FAILED_;

    ck = Protocol_Package_Calculator_checksum((unsigned char*)&cmd_send + 2, cmd_send.len);
    memcpy(data_sent, &cmd_send, cmd_send.len+3);
    data_sent[cmd_send.len+2] = ck;
    Protocol_send_response_data(data_sent, cmd_send.len+3);
    return SUCCESS_;

}

void Protocol_process_data_input(void)
{
		osDelay(1);
	  // return if HEADER get wrong
    if(cmd_process.header != HEADER_16)
    {
        return;
    }
		else{}
		// return if length not corection
    if(cmd_process.len > UART_PACK_SIZE_MAX)
    {
        return;
    }
		else{}
		// calculation checksum and return if checksum fail
    if(Protocol_Package_check_checksum((unsigned char*)&cmd_process + 2, cmd_process.len) != PROTOCOL_TRUE)
    {
        Protocol_UART_send_respond_package(cmd_process.opcode, cmd_process.id, RESPOND_CHECKSUM_ERROR);
        return;
    }
    switch(cmd_process.opcode){
    case CMD_READ_VERSION:
        Protocol_UART_send_respond_package(CMD_READ_VERSION, cmd_process.id, RESPOND_SUCESS);
        break;
    case CMD_JUMP_BOOTLOADER:
        app_info.Is_upgrade = PROTOCOL_TRUE;
        Protocol_UART_send_respond_package(CMD_JUMP_BOOTLOADER, cmd_process.id, RESPOND_SUCESS);
				osDelay(300);
				system_resetMCU();
        break;
		case CMD_TRANSMIT_ADC_CONTROL:
				// if master board => don't send feedback, only process data
				// if slave board => send feedback with roll, pitch, temp, high, status
				#ifdef MASTER_BOARD
					memcpy(&app_slave_infor, cmd_process.payload, sizeof(app_slave_infor));
					if(app_slave_infor.status == PROTOCOL_TRUE)
					{
						app_adc_infor.status = PROTOCOL_TRUE;
					}
					else
						{
							// clear data receive because wrong status
							memset(&app_slave_infor, 0, sizeof(app_slave_infor));
						}
				#else
						Protocol_UART_send_respond_package(CMD_TRANSMIT_ADC_CONTROL, cmd_process.id, RESPOND_SUCESS);
				#endif
				break;
    default:
        break;
    }
}
unsigned char Protocol_Package_check_checksum(unsigned char *packageBuffer, int packageSize){
    unsigned char l_result_uch = 0;
    uint16_t l_countLoop_u16 = 0;
    for(l_countLoop_u16 = 0; l_countLoop_u16 < (packageSize); l_countLoop_u16++){
        l_result_uch += packageBuffer[l_countLoop_u16];
    }
	l_result_uch = (~l_result_uch+1)&0xFF;
	osDelay(1);
	if(l_result_uch == packageBuffer[l_countLoop_u16]){
		return PROTOCOL_TRUE;
	}
	return PROTOCOL_FALSE;
}
unsigned char Protocol_Package_Calculator_checksum(unsigned char *packageBuffer, int packageSize){
    unsigned char result;
    int i;
    result = 0;
    for(i = 0; i<packageSize; i++){
        result += packageBuffer[i];
    }
    return(~result + 1);
}
