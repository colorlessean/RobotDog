int intialsolution[25];
int finalsolution[25];

void Solve(int midpoint, int index){
		SensorType[S1] = sensorColorNxtRED;
		wait1Msec(10); //wait to skip the first line the line going back

		nMotorEncoder[motorA] = 0;

		while(SensorRaw[S1] > midpoint){ //while the value is not black enough
			motor[motorA] = 10; //keep turning right to reach the left path first
			motor[motorC] = -10;
		}
			motor[motorA] = 0;
			motor[motorC] = 0;

			int write = 0;

			if(nMotorEncoder[motorA] < 140){ //turned only 90 degrees to left turn
					write = 1;
			} else if (nMotorEncoder[motorA] < 280){ //turned 180 degrees to go straight
			 		write = 2;
			} else if (nMotorEncoder[motorA] < 420){ // turned 270 degrees to go right
					write = 3;
			} else { //no options turns around and goes back
					write = 4;
			}

			intialsolution[index] = write;

}

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

 	wait1Msec(1000);
}


void Follow(float leftlean, float rightlean){
		SensorType[S1] = sensorColorNxtRED;
		SensorType[S2] = sensorColorNxtRED;

		int tp = 10;
		int kp = 50, ki = 1, kd = 100;
		float leftintegral = 0, rightintegral = 0, leftlastError = 0, rightlastError = 0;
		int count = 0;

		while(nNxtButtonPressed ){
			int left = SensorRaw[S1];
			int right = SensorRaw[S2];

			float lefterror = left - leftlean;
			float righterror = right - rightlean;
			leftintegral = leftintegral + lefterror;
			rightintegral = rightintegral + righterror;
			float leftderivative = lefterror - leftlastError;
			float rightderivative = righterror - rightlastError;

			float turnleft = kp*lefterror + ki*leftintegral + kd*leftderivative;
			float turnright = kp*righterror + ki*rightintegral + kd*rightderivative;

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
				count = count + 1;
				displayString(6, "Intersection %d", count);
				motor[motorA] = 0;
				motor[motorC] = 0;
				wait10Msec(100);

				nMotorEncoder[motorA] = 0;
				nMotorEncoder[motorC] = 0;

				nMotorEncoderTarget[motorA] = 140;
				nMotorEncoderTarget[motorC] = 140;

				motor[motorA] = 10;
				motor[motorC] = 10;

  			wait1Msec(8000);

  			turn180(); //orientates the robot so that it is actually going backwards
  			Solve(leftlean, count);
			}
		}

}

void turn90(bool left){
	int motorpowerA = 10;
	int motorpowerC = -10;
	int motortargetA = 140;
	int motortargetC = -140;
	if (left){}
	else {
		motorpowerA = -10;
		motorpowerC = 10;
		motortargetA = -140;
		motortargetC = 140;
	}
	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorC] = 0;

	nMotorEncoderTarget[motorA] = 140;
	nMotorEncoderTarget[motorC] = -140;

	motor[motorA] = motorpowerA;
	motor[motorC] = -motorpowerC;

 	wait1Msec(1200);
}

void MostEfficientRoute(){
	int previous, current, next;
	for(int i = 0; i < 24; i++){
		previous = intialsolution[i-1];
		current = intialsolution[i];
		next = intialsolution[i+1];
		if (current == 4){
			if(previous == 1 && next == 3){
				current = 4;
			} else if (previous == 1 && next == 2){
				current = 3;
			} else if (previous == 3 && next == 1){
				current = 4;
			} else if (previous == 2 && next == 1){
				current = 3;
			} else if (previous == 2 && next == 2){
				current = 4;
			} else if (previous == 1 && next == 1){
				current = 2;
			}
		 	  previous = 0;
				next = 0;
		}
		finalsolution[i-1] = previous;
		finalsolution[i] = current;
		finalsolution[i+1] = next;

	}
}

void RunEfficiently(float leftlean, float rightlean){
		SensorType[S1] = sensorColorNxtRED;
		SensorType[S2] = sensorColorNxtRED;

		int count = 0;
		int tp = 10;
		int kp = 50, ki = 1, kd = 100;
		float leftintegral = 0, rightintegral = 0, leftlastError = 0, rightlastError = 0;

		while(true){
			int left = SensorRaw[S1];
			int right = SensorRaw[S2];

			float lefterror = left - leftlean;
			float righterror = right - rightlean;
			leftintegral = leftintegral + lefterror;
			rightintegral = rightintegral + righterror;
			float leftderivative = lefterror - leftlastError;
			float rightderivative = righterror - rightlastError;

			float turnleft = kp*lefterror + ki*leftintegral + kd*leftderivative;
			float turnright = kp*righterror + ki*rightintegral + kd*rightderivative;

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
				if (finalsolution[count] == 0){

				}
				else {
					switch(finalsolution[count]){
						case 1: 	turn90(true);
											break;
						case 2: 	turn180();
											break;
						case 3: 	turn90(false);
											break;
						default: break;
 						}
					}
				}
			}

	}



task main(){
		float leftmidpoint = 0, rightmidpoint = 0;
		Calibrate(leftmidpoint, rightmidpoint);

		displayString(5, "gogogogo");
		wait10Msec(100);
		// 1 will be a left turn, 3 will be right turn, 2 will be straight, 4 will turn around

		Follow(leftmidpoint, rightmidpoint);

		//Function for picking up the bone

		MostEfficientRoute();

		RunEfficiently(leftmidpoint, rightmidpoint);

}
