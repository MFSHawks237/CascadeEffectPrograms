#include "drivers/hitechnic-sensormux.h" //HiTechnic Sensor Multiplexer Driver
#include "drivers/hitechnic-protoboard.h" //HiTechnic Prototype Board Driver
#include "drivers/hitechnic-accelerometer.h" //HiTechnic Accelerometer Sensor Driver
#include "drivers/hitechnic-eopd.h" //HiTechnic EOPD Sensor Driver
#include "drivers/hitechnic-irseeker-v2.h" //HiTechnic IR Seeker V2 Sensor Driver
#include "JoystickDriver.c" //Driver to handle joystick input
#include "drivers/gyro.h" //Gyroscopic Sensor Driver
#include "Functions.h"

#define motorSTP 0
#define motorMaxFWD 100
#define motorMaxBKWD -100
#define SrvoArmOUT 120
#define SrvoArmIN 255

#define irpower 15
#define dslope 30/720
#define maxPwr 12
#define dminPwr 7
#define minPwr 32
#define BOUND(n, l, h) (((n) < (l))? (l): ((n) > (h))? (h): (n))
#define drivePower 100
#define rampPWR 50

GYRO  g_Gyro;
float g_turnTarget = 0.0;
bool  g_turnEnabled = false;
float g_tolerance = 0.5;  // needs to be tuned   .5
float Kp = 0.1;           // proportion gain constant needs to be tuned // original .1	//	.05 seems good //	.01 slow but good
bool g_driveEnabled = false;
float g_driveTarget;
float buffer = 5;
bool irseek = true;
int irval;
int i = 0;
float driveError;
int ENCDTMP;
int DES;
float oldDrivePower = 0;

//********//
// Teleop //
//********//

void Drive(tMotor Left, tMotor Right)
{
	//if (abs(joystick.joy1_y1) > 20 || abs(joystick.joy1_y2) > 20)		// Buffer fixes lag
	//{
		motor[Left] = joystick.joy1_y1;		// Left Drive
		motor[Right] = joystick.joy1_y2;		// Right Drive
	//}
	//else
	//{
		//motor[Left] = 0;
		//motor[Right] = 0;
	//}
}

void Arm(tMotor Lift, int i)
{
	if (joy2Btn(2) == false)
	{
		if (joystick.joy2_y1 < 0)
		{
			motor[Lift] = joystick.joy2_y1*.25;
		}
		else
		{
			motor[Lift] = joystick.joy2_y1*.5; //arm stuff
		}
	}
	else
	{
		motor[Lift] = joystick.joy2_y1;
	}
}

void TubeGrab(TServoIndex Tube1, TServoIndex Tube2)
{
	if (joy1Btn(6))
	{
		servo[Tube1] = 155;		// Both servos up
		servo[Tube2] = 110;
	}
	else if (joy1Btn(8))
	{
		servo[Tube1] = 0;			// Both servos down
		servo[Tube2] = 255;
	}
}

/*void Flag (TServoIndex Bar)
{
	if (joy2Btn(4))
	{
		servo[Bar] = 255;
	}
	else if (joy2Btn(2))
	{
		servo[Bar] = 0;
	}
	else if (joy2Btn(3))
	{
		servo[Bar] = 128;
	}
}*/

void Spinner (tMotor spin, TServoIndex Sarm)		// Motor Sweep and Sweep Arm
{
	motor[spin] = joystick.joy2_y2;
	if (joy2Btn(8))
	{
		servo[Sarm] = 10;
	}
	else if (joy2Btn(6))
	{
		servo[Sarm] = 128;
	}
}

//*****************//
// Autonomous Code //
//*****************//

void SetTurnTarget(float angle)
{
	g_turnTarget = GyroGetHeading(g_Gyro) + angle;
	g_turnEnabled = true;
}

void TurnTask()
{
	if (g_turnEnabled)
	{
		float error = g_turnTarget - GyroGetHeading(g_Gyro);
		if (abs(error) > g_tolerance)
		{
			//
			// Simple proportional PID control.
			// Limit the outpout to the range of -100 to 100.
			//
			int turnPower = maxPwr*error*Kp + abs(error)/error*minPwr; // Make power a function of the error
			motor[Left] = turnPower;
			motor[Right] = -turnPower;
		}
		else
		{
			motor[Left] = 0;
			motor[Right] = 0;
			g_turnEnabled = false;
		}
	}
}

void initializeRobot()
{
	GyroInit(g_Gyro, gyro, 0);
	return;
}

void setDistance(float target)
{
	g_driveTarget = target + nMotorEncoder[Right];
	g_driveEnabled = true;
}

void driveForDistance()
{
	driveError = g_driveTarget - nMotorEncoder[Right];
	wait1Msec(25);
	driveError = g_driveTarget - nMotorEncoder[Right];
	if(abs(driveError) > buffer)
	{
		motor[Left] = driveError*dslope + abs(driveError)/driveError*dminPwr;
		motor[Right] = driveError*dslope + abs(driveError)/driveError*dminPwr;
		if (motor[Left] <= 5)
		{
			g_driveEnabled = false;
		}
	}
	else
	{
		motor[Left] = 0;
		motor[Right] = 0;
		g_driveEnabled = false;
	}
}

void driveForDistanceBack()
{
	driveError = g_driveTarget - nMotorEncoder[Right];
	wait1Msec(25);
	driveError = g_driveTarget - nMotorEncoder[Right];
	if(abs(driveError) > buffer)
	{
		motor[Left] = driveError*dslope + abs(driveError)/driveError*dminPwr;
		motor[Right] = driveError*dslope + abs(driveError)/driveError*dminPwr;
		if (motor[Left] >= -5)
		{
			g_driveEnabled = false;
		}
	}
	else
	{
		motor[Left] = 0;
		motor[Right] = 0;
		g_driveEnabled = false;
	}
}

void IRseeker(tSensors IR)
{
	ENCDTMP = nMotorEncoder[Right]; // Save current encoder value
	while(irseek)
	{
		irval = SensorValue[IR];
		if(irval != 5)
		{
			motor[Left] = irpower;
			motor[Right] = irpower;
		}
		else
		{
			irseek = false;
			motor[Left] = 0;
			motor[Right] = 0;
		}
	}
}

void RampGo()
{
	DES = nMotorEncoder[Right] - ENCDTMP; // Get the distance we traveled while seeking the IR
	SetTurnTarget(87.5);
	while(g_turnEnabled) // Turn the robot Perp
	{
		GyroTask(g_Gyro);
		TurnTask();
		wait1Msec(10);
	}
	setDistance(225);
	while(g_driveEnabled) // Drive further from baskets
	{
		driveForDistance();
	}
	SetTurnTarget(87.5);
	while(g_turnEnabled) // Turn the robot around
	{
		GyroTask(g_Gyro);
		TurnTask();
		wait1Msec(10);
	}
	setDistance(DES/1.15);
	while(g_driveEnabled) // Drive back a slightly shorter distance than what we initially traveled
	{
		driveForDistance();
	}
	motor[Left] = 0;
	motor[Right] = 0;
	SetTurnTarget(70.0);
	while(g_turnEnabled) // Turn perpendicular to the ramp
	{
		GyroTask(g_Gyro);
		TurnTask();
		wait1Msec(10);
	}
	wait1Msec(400);
	setDistance(5000);
	while(g_driveEnabled) // Drive to the approximate center of the ramp
	{
		driveForDistance();
	}
	SetTurnTarget(98.5);
	while(g_turnEnabled) // Turn towards the ramp
	{
		GyroTask(g_Gyro);
		TurnTask();
		wait1Msec(10);
	}
	setDistance(5400);
	while(g_driveEnabled) // Drive on to the ramp
	{
		driveForDistance();
	}
}

void turner(int as)
{
	bool one = true;
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

void distCheck ()
{
	if(SensorValue[sonar] >= 20)
	{
		motor[Left] = -10;
		motor[Right] = -80;
		wait1Msec(500);
		motor[Left] = 0;
		motor[Right] = 0;
		deRive(-200);
	}
	else if (SensorValue[sonar] <= 10)
	{
		motor[Left] = -80;
		motor[Right] = -10;
		wait1Msec(500);
		motor[Left] = 0;
		motor[Right] = 0;
		deRive(-200);
	}
}

void negDistCheck ()
{
	if(SensorValue[sonar] >= 20)
	{
		motor[Left] = 80;
		motor[Right] = 10;
		wait1Msec(500);
		motor[Left] = 0;
		motor[Right] = 0;
		deRive(-200);
	}
	else if (SensorValue[sonar] <= 10)
	{
		motor[Left] = 10;
		motor[Right] = 80;
		wait1Msec(500);
		motor[Left] = 0;
		motor[Right] = 0;
		deRive(-200);
	}
}
