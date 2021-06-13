void TravelToDestination(float testx, float testy){
  //left_motor.run(RELEASE);
  //right_motor.run(RELEASE);
  float heading = headingTo(testx,testy);
  //Serial.print("Target Heading ");
  //Serial.print(heading*180/PI);
  float distance = distanceToTarget(testx,testy);

  //then change the motor pwm values to move in the correct location
  //if the desired angle is more positive, it needs to turn left
  //if the desired angle is more negative, it needs to turn right.

  //first start by getting the absolue value of the angle difference. 
  //also if the angle difference is greater than 180, you will want to go the other direction, but that should never be the case.

  float deltaHeading = heading - aj.heading;

  Serial.print("Current Heading: ");
  Serial.print(aj.heading);
  Serial.print(" Desired Heading: ");
  Serial.print(heading);
  Serial.print("Delta Heading: ");
  Serial.println(deltaHeading*180/PI); //converts to degrees
  deltaHeading = deltaHeading*DRIVESMOOTHFACTOR;
  int motorTickDiff = (int)deltaHeading;
  //Serial.print(" motorTickDiff: ");
  //Serial.print(motorTickDiff); //converts to degrees
  int RightMotorValue = motorTickDiff + BASEMOTORSPEED;
  int LeftMotorValue = BASEMOTORSPEED - motorTickDiff;

  //implement slowdown factor
  //get the distance to target.
  float reducefactor = 0;
  if(distance <= 100){
    reducefactor = (100.0 - distance)*SLOWDOWNFACTOR;
  }

  LeftMotorValue -= (int)reducefactor;
  RightMotorValue -= (int)reducefactor;

  //correction for bad input. 
  if(RightMotorValue > 255){
    RightMotorValue = 255;
  }else if(RightMotorValue < 0){
    RightMotorValue = 0;
  }
  if(LeftMotorValue > 255){
    LeftMotorValue = 255;
  }else if(LeftMotorValue < 0){
    LeftMotorValue = 0;
  }
  //Serial.print(" Left motor value: ");
  //Serial.print(LeftMotorValue);
  //Serial.print(" Right motor value: ");
  //Serial.println(RightMotorValue);
  left_motor.setSpeed(LeftMotorValue);
  right_motor.setSpeed(RightMotorValue);

  //if(aj.y > 300){
  //  left_motor.run(RELEASE);
  //  right_motor.run(RELEASE);
  //}

  
}
