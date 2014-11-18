#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     accel,          sensorI2CCustom)
#pragma config(Sensor, S4,     mux,            sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     Left,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     Lift,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     Sweep,         tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    A,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    B,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c";
#include "utils.swoll.h";
#include "drivers/hitechnic-sensormux.h"
#include "drivers/lego-ultrasound.h"
#include "drivers/hitechnic-irseeker-v2.h"

const tMUXSensor sonar1 = msensor_S4_1;
const tMUXSensor sonar2 = msensor_S4_2;
const tMUXSensor IR = msensor_S4_3;

task main()
{
	servo[A] = 0;			// Servo Initialization
	servo[B] = 255;		// Servo Initialization
	while (true)
	{
		int i = 0;					// Does nothing but if removed the code will not work
		i = i+1;						// Does nothing but if removed the code will not work
		Drive(Left, Right);	// Drive
		Arm(Lift);					// Scoring Arm
		TubeGrab(A, B);			// Servo tube base grabbing thing
		/*if (joy1Btn(8))
		{
			servo[A] = 255;
			servo[B] = 0;
		}
		else if (joy1Btn(6))
		{
			servo[A] = 0;
			servo[B] = 255;
		}*/
	}
}
