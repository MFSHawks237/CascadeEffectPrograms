#pragma config(Motor,  motorA,          bob,           tmotorNXT, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
	int joe = -1;
	while(true)
	{
		joe = joe * 2;
		motor[bob] = joe;
		wait1Msec(100);
 	}
}
