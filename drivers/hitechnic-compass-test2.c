#pragma config(Sensor, S1,     HTCOMPASS,           sensorI2CCustom)
#pragma config(Motor,  motorB,          M_RIGHT,       tmotorNormal, PIDControl, reversed)
#pragma config(Motor,  motorC,          M_LEFT,        tmotorNormal, PIDControl, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 * $Id: hitechnic-compass-test2.c 109 2012-09-25 17:10:26Z xander $
 */

/**
 * hitechnic-compass.h provides an API for the HiTechnic Compass Sensor.  This program
 * demonstrates how to use that API to calibrate the compass.
 *
 * The code here is made to work with the standard NXT Tribot.  You will need to edit
 * the WHEELDIST, WHEELSIZE and MOTORSPEED figures to make it work with your robot.
 *
 * Remeber that the robot shouldn't spin more than 360 degrees per 20 seconds.  Also
 * make sure it spins a bit more than 360, perhaps 1 and 1/4 or 1 and 1/2.
 *
 * Changelog:
 * - 0.1: Initial release
 * - 0.2: Added pulsating "*" to indicate progress
 * - 0.3: More comments
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 3.51 AND HIGHER.
 * Xander Soldaat (mightor_at_gmail.com)
 * 25 November 2009
 * version 0.3
 */

#include "drivers/hitechnic-compass.h"

// These measurements are in milimeters.
#define WHEELDIST 115  // distance between the wheels
#define WHEELSIZE 56   // diameter of the wheels

#define MOTORSPEED 4   // speed at which motors should turn

// Lets you know when 20 seconds is over, can help with setting up
// the initial timing and motor speed.
task timeMe() {
  wait1Msec(20000);
  PlaySound(soundBeepBeep);
  while(bSoundActive) EndTimeSlice();
}

// Pulse a big "*" at the bottom of the screen to show that it's
// doing something.
task showPulse() {
  while (true) {
		nxtDisplayCenteredBigTextLine(6, " ");
		wait1Msec(400);
		nxtDisplayCenteredBigTextLine(6, "*");
		wait1Msec(400);
  }
}


// Does some voodoo maths to calculate how many times the wheels should rotate
// to make the robot spin about 360 degrees.
int numRotations() {
  return ((WHEELDIST * 3142) / 1000) / ((WHEELSIZE * 3142) / 1000);
}

// Start the calibration and complain loudly if something goes wrong
void startCalibration() {
  if (!HTMCstartCal(HTCOMPASS)) {
    eraseDisplay();
    nxtDisplayTextLine(1, "ERROR: Couldn't");
    nxtDisplayTextLine(2, "calibrate sensor.");
    nxtDisplayTextLine(4, "Check connection");
    nxtDisplayTextLine(5, "and try again.");
    PlaySound(soundException);
    while(bSoundActive) EndTimeSlice();
    wait1Msec(5000);
    StopAllTasks();
  }
}

// Stop the calibration and complain loudly if somethign goes wrong
void stopCalibration() {
  if (!HTMCstopCal(HTCOMPASS)) {
    eraseDisplay();
    nxtDisplayTextLine(1, "ERROR: Calibration");
    nxtDisplayTextLine(2, "has failed.");
    nxtDisplayTextLine(4, "Check connection");
    nxtDisplayTextLine(5, "and try again.");
    PlaySound(soundException);
    while(bSoundActive) EndTimeSlice();
    wait1Msec(5000);
    StopAllTasks();
  } else {
    nxtDisplayTextLine(1, "SUCCESS: ");
    nxtDisplayTextLine(2, "Calibr. done.");
    PlaySound(soundUpwardTones);
    while(bSoundActive) EndTimeSlice();
    wait1Msec(5000);
  }
}

task main () {
  bFloatDuringInactiveMotorPWM = true;
  int numDegrees = 0;
  nxtDisplayCenteredTextLine(0, "HiTechnic");
  nxtDisplayCenteredBigTextLine(1, "Compass");
  nxtDisplayCenteredTextLine(3, "Test 2");


  nMotorEncoder[M_RIGHT] = 0;
  nMotorEncoder[M_LEFT] = 0;
  // This will make the robot spin about 1.5 times, depends on many factors, YYMV, etc
  numDegrees = ((numRotations() * 3) / 2) * 450;
  StartTask(timeMe);
  startCalibration();
  nxtDisplayCenteredTextLine(5, "Calibrating...");
  StartTask(showPulse);
  motor[M_RIGHT] = MOTORSPEED;
  motor[M_LEFT] = -MOTORSPEED;

  while(nMotorEncoder[M_RIGHT] < numDegrees) wait1Msec(5);
  motor[M_LEFT] = 0;
  motor[M_RIGHT]= 0;
  stopCalibration();
  nxtDisplayCenteredTextLine(5, "Calibration done");
  wait1Msec(5000);
}

/*
 * $Id: hitechnic-compass-test2.c 109 2012-09-25 17:10:26Z xander $
 */