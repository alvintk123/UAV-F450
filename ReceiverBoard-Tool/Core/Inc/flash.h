#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"

#ifndef _FLASH_H
#define _FLASH_H

void saveDataToFlash(int16_t data[], uint16_t data_size);
void loadDataFromFlash(int16_t data[], uint16_t data_size);

#endif //_FLASH_H
