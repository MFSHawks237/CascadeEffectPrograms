#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Sensor, S2,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S3,     accel,          sensorI2CCustom)
#pragma config(Sensor, S4,     mux,            sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     Left,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    A,                    tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    B,                    tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "drivers/hitechnic-sensormux.h"
#include "drivers/lego-ultrasound.h"
#include "drivers/hitechnic-irseeker-v2.h"

const tMUXSensor sonar1 = msensor_S4_1;
const tMUXSensor sonar2 = msensor_S4_2;
const tMUXSensor IR = msensor_S4_3;

task main()
{
	eraseDisplay();
	while (true)
	{
		nxtDisplayTextLine(3, "%d", USreadDist(sonar1));
		nxtDisplayTextLine(5, "%d", USreadDist(sonar2));
		nxtDisplayTextLine(7, "%d", SensorValue[IR]);
		wait1Msec(300);
	}
}
