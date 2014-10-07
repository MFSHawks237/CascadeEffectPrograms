#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S1_C1_1,     FR,            tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     BL,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_1,     FL,            tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     BR,            tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c";

task main()
{
	while(true)
	{
		getJoystickSettings(joystick);
		if (abs(joystick.joy1_x1) > 20 || abs(joystick.joy1_y1) > 20)
		{
			motor[FL] = ((joystick.joy1_y1 + joystick.joy1_x1));
			motor[FR] = ((-joystick.joy1_x1 + joystick.joy1_y1));
			motor[BL] = ((-joystick.joy1_x1 + joystick.joy1_y1));
			motor[BR] = ((joystick.joy1_y1 + joystick.joy1_x1));
		}
		else if (abs(joystick.joy1_x2) > 20)
		{
			motor[FL] = joystick.joy1_x2;
			motor[FR] = -joystick.joy1_x2;
			motor[BL] = joystick.joy1_x2;
			motor[BR] = -joystick.joy1_x2;
		}
		else
		{
			motor[FL] = 0;
			motor[FR] = 0;
			motor[BL] = 0;
			motor[BR] = 0;
		}
	}
}