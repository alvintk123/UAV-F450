#include "joystick.h"

joystickCommandProperties joystickCmd;

 void joystickInitial(void)
 {
	 joystickCmd.rollCommandDeg 	= 0;
	 joystickCmd.pitchCommandDeg 	= 0;
	 joystickCmd.throttleCommand  = 0;
	 joystickCmd.yawCommandDeg		= 0;
 }
 