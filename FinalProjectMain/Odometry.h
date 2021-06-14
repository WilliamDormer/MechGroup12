void init(float*leftDist, float*rightDist, float width){
    noInterrupts(); //i guess this disables interupts for this part
    aj.x=0; //sets the x position to zero
    ac=width; //so ac represents the wheel to wheel width
    *ab=0; 
    aa=leftDist;
    ab=rightDist;
    TCCR1B=8|5;//no clue what this does
    *aa=0;
    TCCR1A=4;
    OCR1A=(int)(100 / 0.064);//timer inverval //0.064 ms/count
    aj.heading=0; //sets the initial heading to zero
    TIMSK1=2;
    interrupts();
    aj.y=0; //sets the y position to zero
  }
  
  void toPlot() {
    Serial.print("x(cm): ");
    Serial.print(aj.x);
    Serial.print("\t y(cm): ");
    Serial.print(aj.y);
    Serial.print("\t z(deg): ");
    Serial.print(aj.heading*180/PI); //converts to degrees
    Serial.println("");
  }

  float distanceToTarget(float x, float y){
    float deltaX = pow(x-aj.x,2.0);
    float deltaY = pow(y-aj.y,2.0);
    return sqrt(deltaX + deltaY);
  }

  float distancePTOP(float x1, float y1,float x2, float y2){
    float deltaX = pow(x2-x1,2.0);
    float deltaY = pow(y2-y1,2.0);
    return sqrt(deltaX + deltaY);
  }
  
  float headingTo(float x, float y) {
  //i think something is wrong with this
    //float deltaX = x-aj.x;
    //float deltaY = y-aj.y;

    //x is the target x location
    //y is the target y location
    //aj.x is the current vehicle x location
    //aj.y is the current vehicle y location
    float val = -atan2((x-aj.x),(y-aj.y));
    //Serial.println(millis());
    //Serial.print("Current heading ");
    //Serial.print(aj.heading);
    //Serial.print(", Target Heading: ");
    //Serial.println(val);
    return val; //x goes first because we are referencing the x axis.
    // 50, 50 should return 1/4 PI
    //  
    
    //return atan2((x-aj.x)*2, y-aj.y);
  }

  ISR(TIMER1_COMPA_vect) {
    //Serial.println("ISR");

    //maybe if i force a time interval this will work better using millis
    
    float af=*aa-ad; //i assume this stuff is to handle the storage while it changes.
    float ag=*ab-ae;
    ad=*aa;
    ae=*ab;
    float ah=(af+ag)/2.0;//d center
    float ai=(ag-af)/ac; //dright - dleft / dbaseline is the change in angle
    aj.x-=ah*sin(aj.heading); //change in x is x + dcenter * sin(current angle)
    aj.y+=ah*cos(aj.heading); //change in y is y + dcenter * cos(current angle)
    aj.heading+=ai; //update the angle
  }

  void countLEncoder(){ // interrupt function for left encoder
  if(millis() - leftTimer > 0){
      leftDistance = leftDistance + ARCPERTICK;
  }
  leftTimer = millis();
      
}

void countREncoder(){ // interrupt function for right encoder
  if(millis() - rightTimer > 0){
      rightDistance = rightDistance + ARCPERTICK;
  }
  rightTimer = millis();
}
