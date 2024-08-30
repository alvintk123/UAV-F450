#ifndef _USER_MAIN_THREAD_H_
#define _USER_MAIN_THREAD_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "protocol.h"

#define Xbee_Master_Pin GPIO_PIN_15
#define Xbee_Master_GPIO_Port GPIOE

void Main_Processing(void);
void sensorFeedback(APPLICATION_SLAVE_INFO *slaveInfo, uint8_t *sensorFlag);
#endif /*_USER_MAIN_THREAD_H_*/
