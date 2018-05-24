int const optimaldistance = 5; //value we will need to find experimentally

double Calibration(double & white){

}

void Grabbing(bool grab){

}

void Follow(double offset){
	int kp = 1000, tp = 50;
	SensorType[S1] = sensorColorNxtFULL;

	int ki = 100;
	double integral = 0;

	int kd = 10000;
	double lastError = 0;

	while (true){ //as long as the robot does not finish the maze ie gets to white
		double LightValue = SensorRaw[S1];
		double error = LightValue - offset;
		integral = integral + error;
		double derivative = error - lastError;
		double Turn = kp*error + ki*integral + kd*derivative;
		Turn = Turn/100;
		motor[motorA] = tp + Turn;
		motor[motorC] = tp - Turn;
		lastError = error;
	}

}

task main()
{
	Follow(251);
}

