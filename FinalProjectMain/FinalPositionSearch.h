  






void turnAndGo(){
  
  float currentTD = rightDistance;
  Serial.print(rightDistance);
  
  while(irPin< 100){
    
    
}


  
  void ForwardAndBackward(){
    
  unsigned long timer = millis();
  
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
