void Calibrate(float & leftmidpoint, float & rightmidpoint)
{
	SensorType[S1] = sensorColorNxtRED;
	SensorType[S2] = sensorColorNxtRED;
	wait10Msec(100);

	displayString(0, "left on black");
	wait10Msec(200);
	float leftblack = SensorRaw[S1];

	displayString(1, "right on black");
	wait10Msec(200);
	float rightblack = SensorRaw[S2];

	displayString(2, "left on white");
	wait10Msec(200);
	float leftwhite = SensorRaw[S1];

	displayString(3, "right on white");
	wait10Msec(200);
	float rightwhite = SensorRaw[S2];

	leftmidpoint = (leftblack + leftwhite)/2;
	rightmidpoint = (rightblack + rightwhite)/2;

}

void turn180(){
	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorC] = 0;

	nMotorEncoderTarget[motorA] = 280;
	nMotorEncoderTarget[motorC] = -280;

	motor[motorA] = 10;
	motor[motorC] = -10;

 	wait1Msec(8000);
}

void Follow(float leftlean, float rightlean){
		SensorType[S1] = sensorColorNxtRED;
		SensorType[S2] = sensorColorNxtRED;
		SensorType[S3] = sensorSONAR;

		int tp = 10;
		float kp = 0.3, ki = 1, kd = 1; //Raul change these
		float leftintegral = 0, rightintegral = 0, leftlastError = 0, rightlastError = 0;
		int count = 0;

		while(SensorValue[S3] > 10){
			float left = SensorRaw[S1];
			float right = SensorRaw[S2];

			float lefterror = left - leftlean;
			float righterror = right - rightlean;
			leftintegral = leftintegral + lefterror;
			rightintegral = rightintegral + righterror;
			float leftderivative = lefterror - leftlastError;
			float rightderivative = righterror - rightlastError;

			float turnleft = kp*lefterror;// + ki*leftintegral + kd*leftderivative;
			float turnright = kp*righterror;// + ki*rightintegral + kd*rightderivative;

			if(left >= leftlean && right >= rightlean){
				motor[motorA] = tp;
				motor[motorC] = tp;
			} else if (left >= leftlean){
				motor[motorA] = tp - turnleft;
				motor[motorC] = tp + turnleft;
			} else if (right >= rightlean){
				motor[motorA] = tp + turnright;
				motor[motorC] = tp - turnright;
			} else {

			}
		}

}



task main()
{
	float leftmidpoint = 0, rightmidpoint = 0;
	Calibrate(leftmidpoint, rightmidpoint);

	displayString(5, "gogogogo");
	wait10Msec(100);
		// 1 will be a left turn, 3 will be right turn, 2 will be straight, 4 will turn around

	Follow(leftmidpoint, rightmidpoint);
}
