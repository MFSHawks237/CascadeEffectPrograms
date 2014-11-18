#pragma config(Sensor, S1,     colour,         sensorI2CCustom)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "drivers/hitechnic-colour-v1.h"

task main()
{
	int color;
	eraseDisplay();
	while(true)
	{
		color = HTCSreadColor(colour);
		nxtDisplayTextLine(4, "Not Red, Blue,");
		nxtDisplayTextLine(5, "%d or Yellow");
		if (color < 0)
		{
			wait1Msec(2000);
			StopAllTasks();
		}
		if (color == 6)
		{
			eraseDisplay();
			nxtDisplayTextLine(4, "Yellow!");
			PlaySound(soundBeepBeep);
			wait1Msec(300);
			eraseDisplay();
		}
		else if (color == 2)
		{
			eraseDisplay();
			nxtDisplayTextLine(4, "Blue!");
			PlaySound(soundDownwardTones);
			wait1Msec(300);
			eraseDisplay();
		}
		else if (color == 9)
		{
			eraseDisplay();
			nxtDisplayTextLine(4, "Red!");
			PlaySound(soundUpwardTones);
			wait1Msec(300);
			eraseDisplay();
		}
	}
}
