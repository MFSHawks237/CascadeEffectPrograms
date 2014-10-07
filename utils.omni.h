#include "drivers/hitechnic-sensormux.h" //HiTechnic Sensor Multiplexer Driver
#include "drivers/hitechnic-protoboard.h" //HiTechnic Prototype Board Driver
#include "drivers/hitechnic-accelerometer.h" //HiTechnic Accelerometer Sensor Driver
#include "drivers/hitechnic-eopd.h" //HiTechnic EOPD Sensor Driver
#include "drivers/hitechnic-irseeker-v2.h" //HiTechnic IR Seeker V2 Sensor Driver
#include "JoystickDriver.c" //Driver to handle joystick input
#include "drivers/gyro.h" //Gyroscopic Sensor Driver
//#include "Functions.h"

#define motorSTP 0
#define motorMaxFWD 100
#define motorMaxBKWD -100
#define SrvoArmOUT 120
#define SrvoArmIN 255

#define irpower 15
#define dslope 30/720
#define maxPwr 10
#define dminPwr 7
#define minPwr 30
#define BOUND(n, l, h) (((n) < (l))? (l): ((n) > (h))? (h): (n))
#define drivePower 100
#define rampPWR 50

GYRO  g_Gyro;
float g_turnTarget = 0.0;
bool  g_turnEnabled = false;
float g_tolerance = 0.5;  // needs to be tuned
float Kp = 0.1;           // proportion gain constant needs to be tuned
bool g_driveEnabled = false;
float g_driveTarget;
float buffer = 5;
bool irseek = true;
int irval;
int i = 0;
float driveError;
int ENCDTMP;
int DES;

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
			//motor[Left] = turnPower;
			//motor[Right] = -turnPower;
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
