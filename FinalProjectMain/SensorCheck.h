void sensorTest(){
  //sensor test
  boolean frontleft = LOW;
  boolean frontright = LOW;
  boolean bottom = LOW;
  
  boolean leftencoder = LOW;
  boolean rightencoder = LOW;
  boolean ultrasonic = LOW;
  
  boolean FlagReached = LOW;

  setPointBottom = 0;
  setPointUltrasonic = 0;

  while(!frontleft || !frontright || !bottom || !leftencoder || !rightencoder || !ultrasonic){
    if(bottom == LOW){ //this just saves memory resources
      ReadBottomIR();
      if(setPointBottom == 0){
        setPointBottom = BottomAverage;
        setPointBottomPerm = setPointBottom;
      } else {
        if(setPointBottom-BottomAverage >IRTHRESHOLD || setPointBottom-BottomAverage < -IRTHRESHOLD){
          bottom = HIGH;
          Serial.println("Bottom Sensor Attached");
        }
      }
    }
    if(frontleft == LOW){
      ReadLeftIR();
      if (LeftAverage == LOW){
        Serial.println("Front Left Sensor Attached");
        frontleft = HIGH;
      }
    }
    if(frontright == LOW){
      ReadRightIR();
      if(RightAverage == LOW){
        frontright = HIGH;
        Serial.println("Front Right Sensor Attached");
      }
    }
    if(leftencoder == LOW){
      if(leftDistance > 0){
        Serial.println("Left Encoder Sensor Attached");
        leftencoder = HIGH;
      }
    }
    if(rightencoder == LOW){
      if(rightDistance > 0){
        Serial.println("Right Encoder Sensor Attached");
        rightencoder = HIGH;
      }
    }

    if(ultrasonic == LOW){
      ReadUltrasonic();
      if(setPointUltrasonic == 0){
        setPointUltrasonic = UltrasonicAverage;
        //Serial.println("Ultrasonic Sensor Initialized");
      } else {
        if(setPointUltrasonic-UltrasonicAverage>30 || setPointUltrasonic-UltrasonicAverage < -30){
          ultrasonic = HIGH;
          Serial.println("UltraSonic Sensor Attached");
        }
      }
    }
  } //end of while loop.  
  Serial.println("Sensor Check Completed.. Waiting 1 second for alignment.");
  delay(1000);

  //need to reset the odometry values. 
  rightDistance = 0;
  leftDistance = 0;
}
