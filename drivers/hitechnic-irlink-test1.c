#pragma config(Sensor, S1,     HTIRL,               sensorI2CCustom)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 * $Id: hitechnic-irlink-test1.c 109 2012-09-25 17:10:26Z xander $
 */

/**
 * hitechnic-irlink.h provides an API for the HiTechnic IR Link Sensor.  This program
 * demonstrates how to use the Power Functions API to control up to 8 motors.
 *
 * Changelog:
 * - 0.1: Initial release
 * - 0.2: More comments
 * - 0.3: Complete rewrite, can demo all three supported modes.
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 3.51 AND HIGHER.
 * Xander Soldaat (mightor_at_gmail.com)
 * 16 September 2010
 * version 0.3
 */

#include "drivers/hitechnic-irlink.h"

/*
  =============================================================================
  Function prototypes

 */
void doPFcomboPwmModeTest(byte speed);
void doPFcomboDirectModeTest(byte speed);
void doPFsinglePinOutputModeTest(byte speed);
void stopMotors();

/*
  =============================================================================
  main task with some testing code

 */
task main {

  // Variables
	byte mode = 0;  // current mode
	byte speed = 0; // current speed

  nxtDisplayCenteredTextLine(0, "HiTechnic");
  nxtDisplayCenteredBigTextLine(1, "IRLink");
  nxtDisplayCenteredTextLine(3, "Test 1");
  nxtDisplayCenteredTextLine(5, "Connect sensor");
  nxtDisplayCenteredTextLine(6, "to S1");
  wait1Msec(2000);
  eraseDisplay();

  nxtDisplayCenteredTextLine(0, "IRLink Test 1");
  nxtDisplayTextLine(3, "Press [ENTER]");
  nxtDisplayTextLine(4, "to switch modes");
  nxtDisplayTextLine(6, "Press < and >");
  nxtDisplayTextLine(7, "to change speed");

  while (true) {
    // Decrease the speed when Left Button is pressed
    if (nNxtButtonPressed == kLeftButton)
      speed--;

    // Increase speed when the Right Button is pressed
		else if (nNxtButtonPressed == kRightButton)
		  speed++;

		// Switch mode when the Enter Button is pressed
		else if (nNxtButtonPressed == kEnterButton) {
		  speed = 0;
		  stopMotors();           // Stop the motors
		  mode++;
		  if (mode > 2) mode = 0; // Make sure mode does not overflow
		}
    // Debounce the button
		while (nNxtButtonPressed != kNoButton) EndTimeSlice();

		// Make sure speed is between 0 and 15
		if (speed < 0) speed = 0;
		else if (speed > 15) speed = 15;

		switch (mode) {
      case 0: doPFcomboPwmModeTest(speed); break;
      case 1: doPFcomboDirectModeTest(speed); break;
      case 2: doPFsinglePinOutputModeTest(speed); break;
    }
    while (nNxtButtonPressed == kNoButton) EndTimeSlice();
  }
}

// Run the motors using the Combo PWM Mode
void doPFcomboPwmModeTest(byte speed) {
  nxtDisplayCenteredTextLine(1, "comboPWM %d", speed);
  PFcomboPwmMode(HTIRL, 0, (ePWMMotorCommand)speed, (ePWMMotorCommand)(15 - speed));
}

// Run the motors using the Combo Direct Mode
void doPFcomboDirectModeTest(byte speed) {
  if (speed > 3) speed = 3;
  nxtDisplayCenteredTextLine(1, "comboDirect %d", speed);
  PFcomboDirectMode(HTIRL, 0, (eCDMMotorCommand)speed, (eCDMMotorCommand)(3 - speed));
}

// Run the motors using the Single Pin Output Mode
void doPFsinglePinOutputModeTest(byte speed) {
  nxtDisplayCenteredTextLine(1, "PFMotor %d", speed);
	PFMotor(pfmotor_S1_C1_A, (ePWMMotorCommand)speed);
	PFMotor(pfmotor_S1_C1_B, (ePWMMotorCommand)(15 - speed));
}

// Stop the motors
void stopMotors() {
  PFcomboDirectMode(HTIRL, 0, CDM_MOTOR_BRAKE, CDM_MOTOR_BRAKE);
}

/*
 * $Id: hitechnic-irlink-test1.c 109 2012-09-25 17:10:26Z xander $
 */
