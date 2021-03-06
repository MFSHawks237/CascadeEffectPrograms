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

#include "JoystickDriver.c"

task main()
{
	servo[A] = -255;
	servo[B] = 255;
	while(true)
	{
		getJoystickSettings(joystick);
		if (joy1Btn(2))
		{
			servo[A] = 240;
			servo[B] = -100;
		}
		else if (joy1Btn(4))
		{
			servo[A] = -255;
			servo[B] = 255;
		}
	}
}
