#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdlib.h"
typedef struct joystickCommandProperties{
    char 	header;

		double 	rollCommandDeg;
		double 	pitchCommandDeg;
		double 	yawCommandDeg;
		double 	throttleCommand;
    
}joystickCommandProperties;



// declare function
void joystickInitial(void);
#endif /* __JOYSTICK_H__ */
