int const arraysize = 50;
int Solution[arraysize];
int Final[arraysize];


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

void Bump(){
				motor[motorA] = 0;
				motor[motorC] = 0;
				wait10Msec(100);

				nMotorEncoder[motorA] = 0;
				nMotorEncoder[motorC] = 0;

				while(nMotorEncoder[motorA] < 100){
					motor[motorA] = 10;
					motor[motorC] = 10;
				}
					motor[motorA] = 0;
					motor[motorC] = 0;
}

void turn180(){
	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorC] = 0;

	nMotorEncoderTarget[motorA] = 280;
	nMotorEncoderTarget[motorC] = -280;

	motor[motorA] = 10;
	motor[motorC] = -10;

 	wait1Msec(3000);
}

void Solve(int midpoint, int index){
		SensorType[S1] = sensorColorNxtRED;
		SensorType[S2] = sensorColorNxtRED;

		nMotorEncoder[motorA] = 0;


		while(SensorRaw[S1] > midpoint && nMotorEncoder[motorA] < 150){ //while the value is not black enough
			motor[motorA] = 10; //keep turning right to reach the left path first
			motor[motorC] = -10;
		}
			motor[motorA] = 0;
			motor[motorC] = 0;

			int write = 0;

			if(nMotorEncoder[motorA] < 150){ //turned only 90 degrees to left turn
					write = 1;
			} else {

				nMotorEncoder[motorC] = 0;

				while (SensorRaw[S2] > midpoint){
					motor[motorA] = -10;
					motor[motorC] = 10;
				}
					motor[motorA] = 0;
					motor[motorC] = 0;

					if(nMotorEncoder[motorC] < 150){//went straight
						write = 2;
					} else if (nMotorEncoder[motorC] < 290){ //went right
						write = 3;
					} else { // went back on itself
						write = 4;
					}
			}

			displayString(7, "Turned: %d", write);

			Solution[index] = write;

}

void Follow(float leftlean, float rightlean){
		SensorType[S1] = sensorColorNxtRED;
		SensorType[S2] = sensorColorNxtRED;
		SensorType[S3] = sensorSONAR;

		int tp = 10;
		float kp = 0.25, ki = 1, kd = 1;
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
				count = count + 1;
				displayString(6, "Intersection %d", count);

				motor[motorA] = 0;
				motor[motorC] = 0;
				wait10Msec(100);

				nMotorEncoder[motorA] = 0;
				nMotorEncoder[motorC] = 0;

				nMotorEncoderTarget[motorA] = 300;
				nMotorEncoderTarget[motorC] = 300;

				motor[motorA] = 10;
				motor[motorC] = 10;

				wait1Msec(1000);

  			Solve(leftlean, count);
			}
		}
		displayString(5, "Done1         ");
}

void ReverseFollow(float leftlean, float rightlean, int finalValue){
		SensorType[S1] = sensorColorNxtRED;
		SensorType[S2] = sensorColorNxtRED;
		SensorType[S3] = sensorSONAR;

		int tp = 10;
		float kp = 0.25, ki = 1, kd = 1;
		float leftintegral = 0, rightintegral = 0, leftlastError = 0, rightlastError = 0;
		int count = 0;

		while(Solution[count] != 0){
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
				displayString(5, "Done3           ");
				int value = count;
				count = count + 1;
				displayString(6, "Intersection %d", count);

				nMotorEncoder[motorA] = 0;
				nMotorEncoder[motorC] = 0;

				if(Solution[value] == 1){
						Bump();

						motor[motorA] = 0;
						motor[motorC] = 0;

						nMotorEncoder[motorA] = 0;
						nMotorEncoder[motorC] = 0;

						nMotorEncoderTarget[motorA] = 140;
						nMotorEncoderTarget[motorC] = -140;

						motor[motorA] = 10;
						motor[motorC] = -10;

					 	wait1Msec(2000);
				} else if (Solution[value] == 3){
						Bump();

						motor[motorA] = 0;
						motor[motorC] = 0;

						nMotorEncoder[motorA] = 0;
						nMotorEncoder[motorC] = 0;

						nMotorEncoderTarget[motorA] = -140;
						nMotorEncoderTarget[motorC] = 140;

						motor[motorA] = -10;
						motor[motorC] = 10;

					 	wait1Msec(2000);
				} else if (Solution[value] == 2){
						nMotorEncoderTarget[motorA] = 100;
						nMotorEncoderTarget[motorC] = 100;

						motor[motorA] = 10;
						motor[motorC] = 10;

						wait1Msec(1000);
				}

			}
		}

}

void Optimization(){

	//array will always have its first value (index = 0) as a do "nothing" '0' value
	//which allows for the previous element to be stored
	//array will also always have a last value as a 0 to allow for the next element to never be null

	for (int i = 0; i < (arraysize-2); i ++){

    int previous, current, next;

		previous = Solution[i];
		current = Solution[i+1];
		next = Solution[i+2];

		if(current == 4){
			if(previous == 1 && next == 3){ //left, back, right
				current = 4; //back
			}
			else if(previous == 1 && next == 2){ //left, back, straight
				current = 3; //right
			}
			else if(previous == 3 && next == 1){ //right, back, left
				current = 4; //back
			}
			else if(previous == 2 && next == 1){ //straight, back, left
				current = 3; //right
			}
			else if(previous == 2 && next == 2){ //straight, back, straight
				current = 4; //back
			}
			else if(previous == 1 && next == 1){ //left, back, left
				current = 2; //straight
			}
                    previous = 0;
                    next = 0;
		}

                Solution[i] = previous;
                Solution[i+1] = current;
                Solution[i+2] = next;

                int index = 0;

                for(int j = 0; j < arraysize; j++){
                    Final[j] = 0;
                }

                for(int k = 0; k < arraysize; k++){
                    if (Solution[k] != 0){
                        Final[index] = Solution[k];
                        index++;
                    }
                }

                for(int l = 0; l < arraysize; l++){
                    Solution[l] = Final[l];
                }
	}
}



task main()
{
	float leftmidpoint = 0, rightmidpoint = 0;
	Calibrate(leftmidpoint, rightmidpoint);

	displayString(5, "gogogogo");
	wait10Msec(1000);
		// 1 will be a left turn, 3 will be right turn, 2 will be straight, 4 will turn around

	Follow(leftmidpoint, rightmidpoint);

	displayString(5, "Done3           ");

	motor[motorA] = 0;
	motor[motorC] = 0;

	for (int i = 0; i < arraysize; i++){
     Optimization();
  }

  int finalValue = 0;

   for (int j = 0; j < arraysize; j++){
       if(Solution[j] != 0){
           finalValue = finalValue+1;

       }
   }

  displayString(0, "Final: %d      ", finalValue);

	turn180();

	//reaches here for sure

  //PickupBone(true);
  	//turn 180 degrees and pick up the bone somehow, orientating the robot to go back



  ReverseFollow(leftmidpoint, rightmidpoint, finalValue);
  	//same linefollowing just when intersections are detected pulls from the solutions array and goes in the direction specified just left-right reversed
  	//when a "flag" (physical barrier) is detected maze is over

  //PickupBone(false);
  	//turns 180 degrees and drops the bone somehow, orientating the robot to start again

  //Victory dance or some shit function

}
