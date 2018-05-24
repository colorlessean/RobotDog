task main()
{
	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorC] = 0;

	nMotorEncoderTarget[motorA] = 140;
	nMotorEncoderTarget[motorC] = -140;

	motor[motorA] = 10;
	motor[motorC] = -10;

 	wait1Msec(8000);
}
