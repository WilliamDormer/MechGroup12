  






void turnAndGo(){
  
  float currentTD = rightDistance;
  Serial.print(rightDistance);
  
  while(irPin< 100){
    
    
  }
}


<<<<<<< Updated upstream
  
  void ForwardAndBackward(){
    
  unsigned long timer = millis();
=======
    if((i-1)!= -1){
       positionArray[i][0] = positionArray[i-1][0] + sin(angle)*currentRadius;//x location
       positionArray[i][1] = positionArray[i-1][1] + cos(angle)*currentRadius; //y location
    }
    else{
       positionArray[i][0] = sin(angle)*currentRadius;//x location
       positionArray[i][1] = cos(angle)*currentRadius; //y location
    }
    
    //Serial.print("X: ");
    //Serial.println(positionArray[i][0]);
    //Serial.print(", Y: ");
    Serial.println(positionArray[i][1]);
 }
 Serial.println("Done");
    
    bool flag = false;
    for(int i = 0; i < arraySize; i++){
      float nextPoint = distanceToTarget(positionArray[i][0], positionArray[i][1]);
      while(nextPoint > 3.0 || flag == true){
        ReadBottomIR();
        nextPoint = distanceToTarget(positionArray[i][0], positionArray[i][1]);
        TravelToDestination(positionArray[i][0], positionArray[i][1]);
          if(setPointBottomPerm - BottomAverage > IRTHRESHOLD || setPointBottomPerm - BottomAverage < -IRTHRESHOLD){
            flag = true;
          }
      }
      if(i < arraySize){
        Serial.println("ERROR: Target not in vicinity.");
        break;
      }
    }  
  //then generate an array of that size. 
>>>>>>> Stashed changes
  
  left_motor.setSpeed(180); // adjust speed values applied to each motor using the updates from the PID controller
  right_motor.setSpeed(180); 
  
  while(millis()<timer + 1000 || irPin < 100){
    left_motor.run(FORWARD);  
    right_motor.run(FORWARD);
  }
  timer = millis();
  while(millis()<timer + 1000){
    left_motor.run(BACKWARD);  
    right_motor.run(BACKWARD);
  }
  }
