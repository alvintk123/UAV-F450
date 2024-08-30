#include "main.h"
#include "string.h"

// declare variable
drone_get_cmd_e 					get_cmd_stt;
drone_cmd_t cmd_send, cmd_get, cmd_process;
PAYLOAD_DRONE_INFOR 		  drone_infor;

uint32_t g_tickstart_u32 = 0, g_ticksByte_u32 = 0;
uint8_t g_rxBuf_u8 = 0;
uint16_t getIndex = 0;
uint8_t g_startProcessData_u8 = PROTOCOL_FALSE;
char data_sent[UART_PACK_SIZE_MAX];

extern APPLICATION_FLASH_INFO 					app_info;
extern APPLICATION_ADC_INFO							app_adc_infor;
extern APPLICATION_SLAVE_INFO		  			app_slave_infor;
extern controlMotor 										Motor;
extern verticalState 										vertState;
extern MS5611_t 												ms5611_1;
//extern MS5611_t 												ms5611_2;
extern GY86_MPU6050_t 									MPU6050; 
//extern bool 														isConfirmSaveHMC5883;
//extern bool 														isConfirmLoadHMC5883;
extern uint32_t os_time;

void Protocol_Initial(void)
{
	// turn on Xbee reset Xbee
	// change to alway communication Xbee-- 
	HAL_GPIO_WritePin(Xbee_GPIO_Port, Xbee_Pin, 1);
	
//	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	
	// init value ADC
	app_slave_infor.status = PROTOCOL_TRUE;
	app_slave_infor.thrustValue 		= 0;
	app_slave_infor.pitchValue 			= 0;
	app_slave_infor.rollValue 			= 0;
	app_slave_infor.yawValue 				= 0;
	app_slave_infor.controlVel 			= 0;
	app_slave_infor.realVel 				= 0;
	app_slave_infor.errorVel 				= 0;
	app_slave_infor.altitude 				= 0;
	app_slave_infor.PitchAngle 			= 0;
	app_slave_infor.RollAngle 			= 0;
	app_slave_infor.YawAngleLevel 	= 0;
	app_slave_infor.PitchRate 			= 0;
	app_slave_infor.RollRate 				= 0;
	app_slave_infor.YawRate 				= 0;
							
//	app_slave_infor.inputThrottle1 	= 0;
//	app_slave_infor.inputThrottle2 	= 0;
//	app_slave_infor.inputThrottle3 	= 0;
//	app_slave_infor.inputThrottle4 	= 0;
		
	app_slave_infor.motorCalibFlag         = 3;
	app_slave_infor.mpu6050CalibFlag       = 3;
	app_slave_infor.ms5611CalibFlag        = 3;
	app_slave_infor.hmc5883lCalibFlag      = 3;
	app_slave_infor.vertVelCalibFlag       = 3;
	app_slave_infor.isStartSensorFlag      = false;
	app_slave_infor.isConnectFlag          = true;
	// if code is loop in Bootloader => we return all information is 00
	strncpy(drone_infor.fw_version, "2.01", 4);
  strncpy(drone_infor.hw_version, "2.01", 4);
  strncpy(drone_infor.Serial_number, "123456789012", 12);
	// change define UART using for project
	HAL_UART_Receive_IT(&huart1, &g_rxBuf_u8, 1);
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    uint8_t l_temp_u8 = 0, *p_u8;
		// using svcRtxKernelGetTickCount instead HAL_GetTick()
		uint32_t l_ticks_u32 = osKernelGetTickCount();
// change define UART using for project
		if(huart->Instance == USART1)
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
    HAL_UART_Transmit_IT(&huart1, (uint8_t *)str_, len);
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
							app_adc_infor.rollControl = g_ADC_DMA_u16[0];
							app_adc_infor.pitchControl = g_ADC_DMA_u16[1];
							app_adc_infor.thrustControl = g_ADC_DMA_u16[2];
							app_adc_infor.yawControl = g_ADC_DMA_u16[3];
							app_adc_infor.status = PROTOCOL_TRUE;
							memcpy(cmd_send.payload, &app_adc_infor, sizeof(app_adc_infor));
							cmd_send.len = 6 + sizeof(app_adc_infor);
						#else
//							app_slave_infor.Ax 										= MPU6050.Ax;
//							app_slave_infor.Ay 										= MPU6050.Ay;
//							app_slave_infor.Az 										= MPU6050.Az;
							
							app_slave_infor.rollValue 						= Motor.rollControl;
							app_slave_infor.pitchValue 						= Motor.pitchControl;
							app_slave_infor.thrustValue 					= Motor.thrustControl;
							app_slave_infor.yawValue 							= Motor.yawControl;
							app_slave_infor.controlVel 						= Motor.verticalVelocityControl;
							app_slave_infor.realVel 							= vertState.velocity;
							app_slave_infor.errorVel 							= Motor.verticalVelocityControl-vertState.velocity;
							app_slave_infor.altitude 							= vertState.altitude; 
							
//							app_slave_infor.PitchAngleRaw					= MPU6050.PitchAngleRaw;
//							app_slave_infor.RollAngleRaw 					= MPU6050.RollAngleRaw;
							app_slave_infor.PitchAngle 						= MPU6050.PitchAngle;
							app_slave_infor.RollAngle 						= MPU6050.RollAngle;
							app_slave_infor.YawAngleLevel 				= MPU6050.YawAngleLevel;
							app_slave_infor.PitchRate 						= MPU6050.pitchRate;
							app_slave_infor.RollRate 							= MPU6050.rollRate;
							app_slave_infor.YawRate 							= MPU6050.yawRate;
//							app_slave_infor.pitchRateOffset 			= MPU6050.pitchRateOffset;
//							app_slave_infor.rollRateOffset 				= MPU6050.rollRateOffset;
//							app_slave_infor.yawRateOffset 				= MPU6050.yawRateOffset;
							app_slave_infor.status 								= PROTOCOL_TRUE;
//							app_slave_infor.isConfirmSaveHMC5883 	= isConfirmSaveHMC5883;
//							app_slave_infor.isConfirmLoadHMC5883 	= isConfirmLoadHMC5883;
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
					memcpy(&app_slave_infor, cmd_send.payload, sizeof(app_slave_infor));
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
						memcpy(&app_adc_infor, cmd_process.payload, sizeof(app_adc_infor));
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
