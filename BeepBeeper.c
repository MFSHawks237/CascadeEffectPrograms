
task main()
{
	int i = 400;
	while(true)
	{
		PlaySound(soundShortBlip);
		wait1Msec(i);
		if (i>1)
			i=i-5;
	}
}
