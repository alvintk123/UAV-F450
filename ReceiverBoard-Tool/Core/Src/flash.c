//#include "stm32f4xx.h"
#include "flash.h"
#include "math.h"

#define FLASH_ADDRESS 0x08010000;
#define DATA_SIZE 8;

static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t FLASH_USER_START_ADDR = 0x08010000;
uint32_t FLASH_USER_END_ADDR = 0x08010050;
uint32_t SectorError = 0;
uint32_t Address = 0;
	

void saveDataToFlash(int16_t data[], uint16_t data_size)
{	
//	uint32_t FLASH_USER_END_ADDR = 0x08060050;
	__IO uint16_t uwData16 = 0;
	__IO uint16_t uwMemoryProgramStatus = 0;
	/* Unlock the Flash *********************************************************/
  /* Enable the flash control register access */
  HAL_FLASH_Unlock();
    
  /* Erase the user Flash area ************************************************/
  /* area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR */
	 /* Clear pending flags (if any) */  
 __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 

  /* Strat the erase operation */
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS; // 
	EraseInitStruct.Sector = FLASH_SECTOR_4; //
	EraseInitStruct.NbSectors = 1; // 
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; //

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  { 
    /* 
      Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
    */

  }
	
	__HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();

  /* Program the user Flash area word by word ********************************/
  /* area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR */

  Address = FLASH_USER_START_ADDR;

	int i = 0;
	uint16_t dataTemp = 0;
	
  while (Address < FLASH_USER_END_ADDR)
  {
		bool isNegative = false;
		if (i > (data_size -1))
		{
			break;
		}
		if (data[i] < 0)
		{
			isNegative = true;
			data[i] 	 = -data[i];
		}
		dataTemp = (uint16_t) (data[i]);
		if (isNegative)
		{
			dataTemp |= (1 << 15);
		}
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, dataTemp) == HAL_OK)
    {
      Address = Address + 2;
			i += 1;
    }
    else
    { 
      /* Error occurred while writing data in Flash memory. 
         User can add here some code to deal with this error */
      /*
        FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
      */
    }
	}

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) */
  HAL_FLASH_Lock(); 

}


void loadDataFromFlash(int16_t data[], uint16_t data_size)
{
	/* Variables ---------------------------------------------------------*/

	
	uint32_t uwAddress = 0;
	int16_t wData16 = 0;
	__IO uint16_t uwData16 = 0;
	int i = 0;
	uwAddress = FLASH_USER_START_ADDR;
	bool isNegative;
  while (uwAddress < FLASH_USER_END_ADDR)
  {
		if (i > (data_size -1))
		{
			break;
		}
		uwData16 = *(__IO uint16_t*)uwAddress;
		isNegative = (uwData16 & (1 << 15)) != 0;
		// Xoa bit dau
		uwData16 &= ~(1 << 15);
		if (isNegative)
		{
			wData16 = -(int16_t) uwData16;
		}
		else
		{
			wData16 = (int16_t) uwData16; 
		}
		data[i] = wData16;
		i++;
		uwAddress += 2;
	}
	
}

