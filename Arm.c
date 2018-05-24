void DogBone(bool grab){ //11 cm away
	//move backward six cm

		motor[motorA] = -10;
		motor[motorC] = -10;
		wait1Msec(1000);
		motor[motorA] = 0;
		motor[motorC] = 0;

	if (grab) {
		nMotorEncoder[motorB] = 0;
		while (nMotorEncoder[motorB] > -70){
			motor[motorB] = -50;
		}
			motor[motorB] = 0;
	} else {
		while (nMotorEncoder[motorB] < 10){
			motor[motorB] = 10;
		}
			motor[motorB] = 0;
	}


}

task main()
{
	DogBone(true);
	motor[motorA] = 10;
	motor[motorC] = -10;
	wait1Msec(1000);
	wait1Msec(1110);
	DogBone(false);
}
