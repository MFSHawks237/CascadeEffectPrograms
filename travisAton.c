#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     accel,          sensorI2CCustom)
#pragma config(Sensor, S4,     mux,            sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     Left,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     Lift,          tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     motorg,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     Sweep,         tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C4_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    A,                    tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    B,                    tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    C,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,                    tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "joystickDriver.c"

task main()
{

	waitForStart();
	servo[C] = 128;
	motor[Lift] = 50;
	wait1Msec(300);
	motor[Lift] = 0;
	motor[Left] = 50;
	motor[Right] = 50;
	wait1Msec(2500);


}
