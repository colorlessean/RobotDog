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

void Follow(float leftlean, float rightlean){
		SensorType[S1] = sensorColorNxtRED;
	int kp = 1000, tp = 10;

	int ki = 100;
	double integral = 0;

	int kd = 10000;
	double lastError = 0;

	while (true){ //as long as the robot does not finish the maze ie gets to white
		double LightValue = SensorRaw[S1];
		double error = LightValue - leftlean;
		integral = integral + error;
		double derivative = error - lastError;
		double Turn = kp*error + ki*integral + kd*derivative;
		Turn = Turn/100;
		motor[motorA] = tp + Turn;
		motor[motorC] = tp - Turn;
		lastError = error;
	}




}

task main(){
		float leftmidpoint = 0, rightmidpoint = 0;
		Calibrate(leftmidpoint, rightmidpoint);

		displayString(5, "gogogogo");
		wait10Msec(100);

		Follow(leftmidpoint, rightmidpoint);

}
