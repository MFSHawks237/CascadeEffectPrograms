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
#pragma config(Servo,  srvo_S1_C3_4,    D,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    C,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,                    tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


task main()
{
	while(true)
	{
		servo[D] = 253;
		wait1Msec(1000);
		servo[D] = 0;
		wait1Msec(1000);
	}
}