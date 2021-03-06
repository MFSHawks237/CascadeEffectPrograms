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

task main()
{
	deRive(-2900);
}
