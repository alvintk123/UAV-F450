#ifndef _USER_TOOLCOMMUNICATION_THREAD_H_
#define _USER_TOOLCOMMUNICATION_THREAD_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "stdbool.h"
#include "protocol.h"
#include "joystick.h"
void ToolCommunication_Initization(void);
void ToolCommunication_Processing(void);
#endif /*_USER_TOOLCOMMUNICATION_THREAD_H_*/
