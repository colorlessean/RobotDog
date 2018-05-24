int const arraysize = 50; //array needs to be large enough to at least hold 50 decisions
int Solution[arraysize]; //arrays are set up to be the arraysize
												 //solution is for the solutions and Copy is for copying to before refining and copying back (contained in optimazation function)


void Calibrate(float & leftmidpoint, float & rightmidpoint){ //Calibration function calibrates black and white values for the sensors to use a maximum and minimum intensity values
	SensorType[S1] = sensorColorNxtRED; //sets the two sensors to being RED LED output
	SensorType[S2] = sensorColorNxtRED;
	wait10Msec(100);

	displayString(0, "left on black"); //takes the value for black for each sensor
	wait10Msec(200);
	float leftblack = SensorRaw[S1]; //sensorRaw takes intensity as opposed to actual color

	displayString(1, "right on black");
	wait10Msec(200);
	float rightblack = SensorRaw[S2];

	displayString(2, "left on white"); //takes the value for white for each sensor
	wait10Msec(200);
	float leftwhite = SensorRaw[S1];

	displayString(3, "right on white");
	wait10Msec(200);
	float rightwhite = SensorRaw[S2];

	leftmidpoint = (leftblack + leftwhite)/2; //take the values for black and white for each sensor and then averages them to get a midpoint value where there will be no correction nesseary
	rightmidpoint = (rightblack + rightwhite)/2;

}

void Bump(){ //moves the robot forward a little to allow for it to not see an intersection twice when trying to go straight from an intersection
	motor[motorA] = 0;
	motor[motorC] = 0;
	wait10Msec(100);

	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorC] = 0;

	while(nMotorEncoder[motorA] < 100){ //for a small rotation it moves it forward
		motor[motorA] = 10;
		motor[motorC] = 10;
	}
	motor[motorA] = 0; //then the robot stops
	motor[motorC] = 0;
}

void turn180(){ //turns robot 180 degrees using motor encoders
	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorC] = 0;

	nMotorEncoderTarget[motorA] = 280;
	nMotorEncoderTarget[motorC] = -280;

	motor[motorA] = 10;
	motor[motorC] = -10;

 	wait1Msec(3000);
}

void Solve(int midpoint, int index){ //starts whenever an intersection is reached to make the descisions on turning and store the direction taken
	SensorType[S1] = sensorColorNxtRED; //set up sensors to read intensity
	SensorType[S2] = sensorColorNxtRED;

	nMotorEncoder[motorA] = 0; //reset motor encoder for turns

	while(SensorRaw[S1] > midpoint && nMotorEncoder[motorA] < 150){ //while the value is not black enough && the robot hasn't turned more than 90 degrees left
		motor[motorA] = 10; //turn left to reach for left
		motor[motorC] = -10;
	}
	motor[motorA] = 0;
	motor[motorC] = 0;

	int write = 0; //set up variable to be written to the array

	if(nMotorEncoder[motorA] < 150){ //found a path on the left direction sweep
		write = 1;
	} else { //found a path on the right direction sweep
		nMotorEncoder[motorC] = 0; //uses the c motor rather than a which is already counting

		while (SensorRaw[S2] > midpoint){ //while the left sensor does not see the line so that it can stradle the line when it does as opposed to sensor1 where the robot would not be lined up
			motor[motorA] = -10;
			motor[motorC] = 10;
		}
		motor[motorA] = 0;
		motor[motorC] = 0;

		if(nMotorEncoder[motorC] < 150){// turned 180 therefore went straight
  		write = 2;
		} else if (nMotorEncoder[motorC] < 290){ // turned 270 therefore went right
			write = 3;
		} else { // turned around 360 therefore the robot turned fully around
			write = 4;
		}
 }
	displayString(7, "Turned: %d", write); //readout line
	Solution[index] = write; //actually writing to the array
}

void Follow(float leftlean, float rightlean){ //the PID which uses two sensors and stradles the line rather than following an edge
	SensorType[S1] = sensorColorNxtRED; //setting up the sensors to see the line
	SensorType[S2] = sensorColorNxtRED;
	SensorType[S3] = sensorSONAR; // sets up the sonar to allow it to stop when seeing the "dogbone"

	int tp = 10; //speed of the motor before modification
	float kp = 0.25, ki = 0.01, kd = 0.01; //series of multipliers: kp is for the proportional calculation, ki for the integral calculation, kd for the derivative calculation
	float leftintegral = 0, rightintegral = 0, leftlastError = 0, rightlastError = 0; //setup values for the intergral and the lastError is for the derivative calculations
	int count = 0; //runs a counter for the sake of the array which will be passed into the solving function later

	while(SensorValue[S3] > 40){ //while the robot has not detected the dogbone/end of maze
		float left = SensorRaw[S1]; //takes in the intensity values of each sensor
		float right = SensorRaw[S2];

		float lefterror = left - leftlean; //error is the amount the sensor sees too much white or black and makes the correction based off this
		float righterror = right - rightlean; //does it for both sensors
		leftintegral = leftintegral + lefterror; //integral counts up all the errors such that small corrects which would be missed end up being corrected as they accululate
		rightintegral = rightintegral + righterror;
		float leftderivative = lefterror - leftlastError; //derivative calculation tries to predict future error based on current error to smoothen future correction
		float rightderivative = righterror - rightlastError;

		float turnleft = kp*lefterror + ki*leftintegral + kd*leftderivative; //the values by which the motorw will actually be changed left with left, right with right
		float turnright = kp*righterror + ki*rightintegral + kd*rightderivative;

		if(left >= leftlean && right >= rightlean){ //if the value is whiter than the midvalue then the line is between the sensors it can go straight
			motor[motorA] = tp;
			motor[motorC] = tp;
		} else if (left >= leftlean){ //left sensor sees too much black and so must turn left to push the line between the sensors again
			motor[motorA] = tp - turnleft;
			motor[motorC] = tp + turnleft;
		} else if (right >= rightlean){//alagous to left just with right sensor and right turning
			motor[motorA] = tp + turnright;
			motor[motorC] = tp - turnright;
		} else { //however when it sees too much black in both sensors it means it has reached an intersection
			count = count + 1; //the count of the intersection goes up one value to count the amoutn of intersections passed

			motor[motorA] = 0; //stops the robot on the intersection
			motor[motorC] = 0;
			wait10Msec(100);

			nMotorEncoder[motorA] = 0; //resets motor encoders
			nMotorEncoder[motorC] = 0;

			nMotorEncoderTarget[motorA] = 300; //distance needed to be rotating on the axis of rotation of the wheels to make clean turns
			nMotorEncoderTarget[motorC] = 300;

			motor[motorA] = 10; //moves the robot to the position needed
			motor[motorC] = 10;

			wait1Msec(1000);

 			Solve(leftlean, count); //runs the decision making algorithm which also writes to the array the value of what turn it made
		}
	}
}

void ReverseFollow(float leftlean, float rightlean, int finalValue){ //Runs the same PID code just takes intersections very differently with array values being red
	SensorType[S1] = sensorColorNxtRED;
	SensorType[S2] = sensorColorNxtRED;
	SensorType[S3] = sensorSONAR;

	int tp = 10;
	float kp = 0.25, ki = 0.01, kd = 0.01;
	float leftintegral = 0, rightintegral = 0, leftlastError = 0, rightlastError = 0;
	int count = 0;

	while(Solution[count] != 0){ //as long as the array is not done runs the dual Sensor PID
		float left = SensorRaw[S1];
		float right = SensorRaw[S2];

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
		} else { //reaches an intersection


			nMotorEncoder[motorA] = 0; //resets encoders
			nMotorEncoder[motorC] = 0;

			if(Solution[count] == 1){ //turns left 90 degrees
				Bump(); //moves it up off the intersection

				motor[motorA] = 0; //turns the robot 90 degrees left
				motor[motorC] = 0;

				nMotorEncoder[motorA] = 0;
				nMotorEncoder[motorC] = 0;

				nMotorEncoderTarget[motorA] = 140; //target value to rotate
				nMotorEncoderTarget[motorC] = -140;

				motor[motorA] = 10; //rotates left
				motor[motorC] = -10;

			 	wait1Msec(2000); //duraction allowed to rotate
			} else if (Solution[count] == 3){ //turns right
				Bump(); //moves up off the intersection

				motor[motorA] = 0;
				motor[motorC] = 0;

				nMotorEncoder[motorA] = 0;
				nMotorEncoder[motorC] = 0;

				nMotorEncoderTarget[motorA] = -140; //90 degrees exact opposite
				nMotorEncoderTarget[motorC] = 140;

				motor[motorA] = -10;
				motor[motorC] = 10;

			 	wait1Msec(2000);
			} else if (Solution[count] == 2){ //just goes straight
				nMotorEncoderTarget[motorA] = 100; //moves it up a little
				nMotorEncoderTarget[motorC] = 100;

				motor[motorA] = 10;
				motor[motorC] = 10;

				wait1Msec(1000);
			}

			count = count + 1; //adds to the count value to show an intersection

		}
	}
}

void Optimization(){
	int Copy[arraysize];
	//array will always have its first value (index = 0) as a do "nothing" '0' value
	//which allows for the previous element to be stored
	//array will also always have a last value as a 0 to allow for the next element to never be null

	for (int i = 0; i < (arraysize-2); i ++){ //arraysize -2 because the arrays first and last values are 0s

    int previous, current, next; //set the values of previous, current and next

		previous = Solution[i]; //takes the values of previous, current, and next
		current = Solution[i+1];
		next = Solution[i+2];

		if(current == 4){ // if the robot turned around
			if(previous == 1 && next == 3){ //left, back, right
				current = 4; //optimizes to back
			}
			else if(previous == 1 && next == 2){ //left, back, straight
				current = 3; //optimizes to right
			}
			else if(previous == 3 && next == 1){ //right, back, left
				current = 4; //optimizes to back
			}
			else if(previous == 2 && next == 1){ //straight, back, left
				current = 3; //optimizes to right
			}
			else if(previous == 2 && next == 2){ //straight, back, straight
				current = 4; //optimizes to back
			}
			else if(previous == 1 && next == 1){ //left, back, left
				current = 2; //optimizes to straight
			}
      previous = 0; //sets the values to dead values of zero so they can later be removed
      next = 0;
		}

    Solution[i] = previous; //reads to the next values of the array
    Solution[i+1] = current;
    Solution[i+2] = next;

    int index = 0;

    for(int j = 0; j < arraysize; j++){
     Copy[j] = 0; //intializes array copy
    }

    for(int k = 0; k < arraysize; k++){
     if (Solution[k] != 0){ //if the value scanned and is not a zero it gets copied over to the correct new index
     	Copy[index] = Solution[k]; //copys the non-zero to the new array at a new compressed index
     	index++; //index goes up everytime a non-zero value is copied
     } //if a zero it is ignored
    }

    for(int l = 0; l < arraysize; l++){
     Solution[l] = Copy[l]; // copies back all the values to the solution array for use in other funtions
    }
	}
}

void DogBone(bool grab){ //turns around 180 degrees and then picks it up or puts down based on boolean value
  turn180();
  motor[motorA] = -10; //moves claw toward the bone to actaully grab it
  motor[motorC] = -10;
  wait1Msec(1000);
	motor[motorA] = 0;
	motor[motorC] = 0;

  if (grab) { //picks up
  	nMotorEncoder[motorB] = 0;
	  while (nMotorEncoder[motorB] > -70){
			motor[motorB] = -50;
	  }
		motor[motorB] = 0;
	} else { //puts down
		while (nMotorEncoder[motorB] < 10){
			motor[motorB] = 10;
		}
		motor[motorB] = 0; //stops the claw
	}
}

task main(){
	float leftmidpoint = 0, rightmidpoint = 0; //intializes values
	Calibrate(leftmidpoint, rightmidpoint); //runs the calibration function

	displayString(5, "gogogogo"); //tells user to let the robot go freely
	wait10Msec(1000); //gives the user a second

	Follow(leftmidpoint, rightmidpoint); //runs the following algorithm
	motor[motorA] = 0;
	motor[motorC] = 0;

	for (int i = 0; i < arraysize; i++){ //optimizes multiple times to compress to absolute shortest route
     Optimization();
  }

  int finalValue = 0; //intializes value

  for (int j = 0; j < arraysize; j++){ //finds the last non-zero instruciton in the array so the robot only reads till there
  	if(Solution[j] != 0){
      finalValue = finalValue+1;
    }
  }
  DogBone(true); // picks up bone
  ReverseFollow(leftmidpoint, rightmidpoint, finalValue); //goes back through the maze
  DogBone(false); //drops off bone
}
