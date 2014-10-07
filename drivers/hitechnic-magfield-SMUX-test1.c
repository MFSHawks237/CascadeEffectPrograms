#pragma config(Sensor, S1,     HTSMUX,              sensorI2CCustom)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 * $Id: hitechnic-magfield-SMUX-test1.c 109 2012-09-25 17:10:26Z xander $
 */

/**
 * HTGYRO-driver.h provides an API for the HiTechnic Magnetic Field Sensor.  This program
 * demonstrates how to use that API in combination with the Sensor MUX.
 *
 * Changelog:
 * - 0.1: Initial release
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 3.51 AND HIGHER.
 * Xander Soldaat (mightor_at_gmail.com)
 * 27 July 2010
 * version 0.1
 */

#include "drivers/hitechnic-sensormux.h"
#include "drivers/hitechnic-magfield.h"

// The sensor is connected to the first port
// of the SMUX which is connected to the NXT port S1.
// To access that sensor, we must use msensor_S1_1.  If the sensor
// were connected to 3rd port of the SMUX connected to the NXT port S4,
// we would use msensor_S4_3

// Give the sensor a nice easy to use name
const tMUXSensor HTMAG = msensor_S1_1;

task main () {

  nxtDisplayCenteredTextLine(0, "HiTechnic");
  nxtDisplayCenteredBigTextLine(1, "MAGNETIC");
  nxtDisplayCenteredTextLine(3, "Field Sensor");
  nxtDisplayCenteredTextLine(4, "SMUX Test");
  nxtDisplayCenteredTextLine(5, "Connect SMUX to");
  nxtDisplayCenteredTextLine(6, "S1 and sensor to");
  nxtDisplayCenteredTextLine(7, "SMUX Port 1");
  wait1Msec(2000);

  eraseDisplay();
  time1[T1] = 0;
  while(true) {
    eraseDisplay();
    nxtDisplayTextLine(1, "Resetting");
    nxtDisplayTextLine(2, "bias");
    wait1Msec(500);

    // Start the calibration and display the offset
    nxtDisplayTextLine(2, "Bias: %4d", HTMAGstartCal(HTMAG));
    PlaySound(soundBlip);
    while(bSoundActive) EndTimeSlice();
    while(nNxtButtonPressed != kNoButton) EndTimeSlice();

    while(nNxtButtonPressed != kEnterButton) {
      eraseDisplay();

      nxtDisplayTextLine(1, "Reading");
      // Read the current calibration offset and display it
      nxtDisplayTextLine(2, "Bias: %4d", HTMAGreadCal(HTMAG));

      nxtDisplayClearTextLine(4);
      // Read the current rotational speed and display it
      nxtDisplayTextLine(4, "Mag:   %4d", HTMAGreadVal(HTMAG));
      nxtDisplayTextLine(6, "Press enter");
      nxtDisplayTextLine(7, "to recalibrate");
      wait1Msec(100);
    }
  }
}

/*
 * $Id: hitechnic-magfield-SMUX-test1.c 109 2012-09-25 17:10:26Z xander $
 */