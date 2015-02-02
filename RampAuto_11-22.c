#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     accel,          sensorI2CCustom)
#pragma config(Sensor, S4,     mux,            sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     Left,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     Lift,          tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     Sweep,         tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    A,                    tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    B,                    tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    C,                    tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

///////////////////////////////////
///////////////////////////////////
///			Last Edit: Armon	10/23 ///
///////////////////////////////////
///////////////////////////////////


#include "drivers/hitechnic-sensormux.h"
#include "drivers/lego-ultrasound.h"
#include "drivers/hitechnic-irseeker-v2.h"

const tMUXSensor sonar1 = msensor_S4_1;
const tMUXSensor sonar2 = msensor_S4_2;
const tMUXSensor IR = msensor_S4_3;

#include "utils.swoll.h"

bool one = true; // while toggle boolean

void deRive(int disr)
{
	setDistance(disr);
	while(g_driveEnabled)
	{
		/*if (USreadDist(sonar1) < 35 && USreadDist(sonar2) < 35)
		{
		PlaySound(soundBeepBeep);
		wait1Msec(100);
		g_driveEnabled = false;
		}*/
		if (disr > 0)
		{
			driveForDistance();
		}
		else if (disr < 0)
		{
			driveForDistanceBack();
		}
	}
	motor[Left] = 0;
	motor[Right] = 0;
}

void driveAfterTurn(int disr)
{
	int i = 0;
	setDistance(disr);
	while(g_driveEnabled)
	{
		i += 1;
		if (USreadDist(sonar1) < 35 && USreadDist(sonar2) < 35)
		{
			PlaySound(soundBeepBeep);
			wait1Msec(100);
			if (abs(driveError) > buffer)
			{
				g_driveEnabled = false;
				if (i < 300)
				{
					StopAllTasks();
				}
			}
		}
		driveForDistance();
	}
	motor[Left] = 0;
	motor[Right] = 0;
}

void turner(int as)
{
	while (one)
	{
		initializeRobot();
		SetTurnTarget(as);			//good target	83.5 for 90	//40 for 45
		while(g_turnEnabled) // Turn the robot Perp
		{
			GyroTask(g_Gyro);
			TurnTask();
			wait1Msec(10);
		}
		wait1Msec(1000);
		one = false;
	}
	one = true;
}

task main()
{
	waitForStart();
	servo[C] = 0;
	motor[Lift] = 50;
	wait1Msec(300);
	motor[Lift] = 0;
	int pitch = 0; // x-axis
	int bank = 0; // y-axis
	int z = 0; // z-axis
	string _tmp;
	eraseDisplay();
	motor[Left] = 50;
	motor[Right] = 60;
	wait1Msec(1500);
	motor[Left] = 0;
	motor[Right] = 0;
	while(one)
	{
		//eraseDisplay();
		HTACreadAllAxes(accel, pitch, bank, z);
		nxtDisplayTextLine(2, "   X    Y    Z");
		StringFormat(_tmp, "%4d %4d", pitch, bank);
		nxtDisplayTextLine(3, "%s %4d", _tmp, z);
		if (pitch <= -19 || pitch >= 19)
		{
			HTACreadAllAxes(accel, pitch, bank, z);
			wait1Msec(1);
			if (pitch <= -19 || pitch >= 19)
			{
				HTACreadAllAxes(accel, pitch, bank, z);
				wait1Msec(1);
				if (pitch <= -19 || pitch >= 19)
				{
					motor[Left] = 15;
					motor[Right] = 40;	// >=19 works
				}
				else
				{
					HTACreadAllAxes(accel, pitch, bank, z);
					motor[Left] = 0;
					motor[Right] = 0;
				}
			}
		}
		else
		{
			nMotorEncoder[Right] = 0;
			motor[Left] = 50;
			motor[Right] = 50;
			wait1Msec(500);
			motor[Left] =0;
			motor[Right] = 0;
			one = false;
		}
	}
	one = true;
	//turner(-180);	// Turn 1		-83.5
	servo[A] = 240;
	servo[B] = 0;
	deRive(-2000);			//2900`	//-4000
	wait1Msec(100);
	servo[A] = 0;
	servo[B] = 255;
	wait1Msec(400);
	deRive(nMotorEncoder[Right]);
	//deRive(1500);
	/*turner(-40);	// Turn 2
	driveAfterTurn(3300);
	turner(40);	// Turn 3
	driveAfterTurn(1900);
	turner(40);	// Turn 4
	driveAfterTurn(1500);*/
}
