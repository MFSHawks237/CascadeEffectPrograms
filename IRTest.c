#pragma config(Sensor, S4,     IR,             sensorHiTechnicIRSeeker1200)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

int irval;

task main()
{
	while(true)
	{
		eraseDisplay();
			irval = SensorValue[IR];
			nxtDisplayString( 1, "%d", irval);

	}
}
