#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     accel,          sensorI2CCustom)
#pragma config(Sensor, S4,     mux,            sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     Left,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     Lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     Sweep,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    Tube1,                tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    Tube2,                tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    Sarm,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
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
	waitForStart();
	servo[Tube1] = 0;			// Servo Initialization
	servo[Tube2] = 255;		// Servo Initialization
	while (true)
	{
		Drive(Left, Right);		// Drive
		Arm(Lift, i);					// Scoring Arm
		TubeGrab(Tube1, Tube2);				// Servo tube base grabbing thing
		Spinner(Sweep, Sarm);		// Sweep motor control and sweep servo
	}
}
