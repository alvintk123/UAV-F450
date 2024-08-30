#ifndef _USER_RECEIVERCOMMUNICATION_THREAD_H_
#define _USER_RECEIVERCOMMUNICATION_THREAD_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "stdbool.h"
#include "protocol.h"

//void IMU_Initization(void);
void ReceiverCommunication_Processing(void);
#endif /*_USER_RECEIVERCOMMUNICATION_THREAD_H_*/
