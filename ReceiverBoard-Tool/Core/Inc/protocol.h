#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f4xx_hal.h"
#include "stm32_hal_legacy.h"
#include "ControlMotor.h"
#include "ms5611.h"

#define CRC_WRONG                           0
#define SUCCESS_                            1
#define FAILED_                           	2
#define PROTOCOL_FALSE											0
#define PROTOCOL_TRUE												1

#define UART_PACK_SIZE_MAX           				520

#define CMD_JUMP_BOOTLOADER         				0x10
#define CMD_START_UPGRADE           				0x11
#define CMD_UPGRADE_RUNNING         				0x12
#define CMD_UPGRADE_FINNIHED        				0x13
#define CMD_READ_VERSION										0x14
#define CMD_TRANSMIT_ADC_CONTROL						0x15


#define HEADER0         										0x55
#define HEADER1         										0xFF
#define TIMEOUT_GET_BYTE     								156
#define HEADER_16         								  0x55FF


#define RESPOND_SUCESS              				0x01
#define RESPOND_FAILED              				0x02
#define RESPOND_UNSUPPORT           				0x03    // Command is not supported
#define RESPOND_CHECKSUM_ERROR      				0x04    // Checksum error
#define RESPOND_BUSY                				0x05    // Machine busy
#define RESPOND_FAILED_CHECKSUM     				0x09

#define RESPONSE_PACKAGE										0x02


#pragma pack(1)
typedef enum drone_get_cmd_e{
    DRONE_GET_HEADER0 = 0,
    DRONE_GET_HEADER1,
    DRONE_GET_LEN0,
    DRONE_GET_LEN1,
    DRONE_GET_TYPE,
    DRONE_GET_TRAN_ID0,
    DRONE_GET_TRAN_ID1,
    DRONE_GET_CMD_CODE,
    DRONE_GET_OTHER
}drone_get_cmd_e;


typedef struct drone_cmd_t{
    uint16_t 	header;
    uint16_t 	len;
    uint8_t 	type;
    uint16_t 	id;
    uint8_t 	opcode;
    uint8_t 	payload[263];
}drone_cmd_t;

typedef struct PAYLOAD_DRONE_INFOR{
    unsigned char     Status;
    char fw_version[4];
    char hw_version[4];
    char Serial_number[12];
}PAYLOAD_DRONE_INFOR;
typedef struct Payload_upgrade_begin
{
    uint32_t 	Total_length;
    uint32_t	CRC_32;
}Payload_upgrade_begin;

typedef struct Payload_upgrade_running
{
    uint32_t 	Sent_length;
    uint16_t 	Length;
    uint8_t   Data[256];
}Payload_upgrade_running;

#pragma pack()

// extern data

extern uint8_t g_startProcessData_u8;
extern UART_HandleTypeDef huart1;

// declare function
void Protocol_Initial(void);
void Protocol_send_response_data(char *str_, uint16_t len);
void Protocol_process_data_input(void);
unsigned char Protocol_UART_send_respond_package(unsigned char cmd, uint16_t ID, unsigned char status);
unsigned char Protocol_Package_check_checksum(unsigned char *packageBuffer, int packageSize);
unsigned char Protocol_Package_Calculator_checksum(unsigned char *packageBuffer, int packageSize);

#endif /* __PROTOCOL_H__ */
