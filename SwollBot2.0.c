#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S2,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     accel,          sensorI2CCustom)
#pragma config(Sensor, S4,     sonar,          sensorSONAR)
#pragma config(Motor,  mtr_S1_C1_1,     Left,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    A,                    tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    B,                    tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


#include "JoystickDriver.c";

task main()
{
	servo[A] = 0;
	servo[B] = 255;
	while(true)
	{
		getJoystickSettings(joystick);
		if (abs(joystick.joy1_y1) > 10)
		{
			motor[Left] = joystick.joy1_y1; //Left
		}
		else
		{
			motor[Left] = 0
		}
		if (abs(joystick.joy1_y2) > 10)
		{
			motor[Right] = joystick.joy1_y2; //Right
		}
		else
		{
			motor[Right] = 0
		}
		if (joy1Btn(8))
		{
			servo[A] = 240;
			servo[B] = 0;
		}
		else if (joy1Btn(6))
		{
			servo[A] = 0;
			servo[B] = 255;
		}
	}
}
