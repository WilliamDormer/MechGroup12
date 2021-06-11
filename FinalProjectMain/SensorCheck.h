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

  for(int i = 0; i<25; i++){ //this is required to warm up the sensors for some reason, presumably so the averages get more accurate.
    CollectSensorData();
    delay(5);
  }
  
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
      if (LeftAverage == HIGH){
        Serial.println("Front Left Sensor Attached");
        frontleft = HIGH;
      }
    }
    if(frontright == LOW){
      ReadRightIR();
      if(RightAverage == HIGH){
        frontright = HIGH;
        Serial.println("Front Right Sensor Attached");
      }
    }
    if(leftencoder == LOW){
      if(LeftEncoderCount > 0){
        Serial.println("Left Encoder Sensor Attached");
        leftencoder = HIGH;
      }
    }
    if(rightencoder == LOW){
      if(RightEncoderCount > 0){
        Serial.println("Right Encoder Sensor Attached");
        rightencoder = HIGH;
      }
    }

    if(ultrasonic == LOW){
      ReadUltrasonic();
      if(setPointUltrasonic == 0){
        setPointUltrasonic = UltrasonicAverage;
      } else {
        if(setPointUltrasonic-UltrasonicAverage>10 || setPointUltrasonic-UltrasonicAverage < -10){
          ultrasonic = HIGH;
          Serial.println("UltraSonic Sensor Attached");
        }
      }
    }
  } //end of while loop.  
  Serial.println("Sensor Check Completed");
}
