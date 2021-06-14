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
    Serial.print("x(m): ");
    Serial.print(aj.x);
    Serial.print("\t y(m): ");
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
  /*
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
  */

  ISR(TIMER1_COMPA_vect) {
    //Serial.println("ISR");
    unsigned long timer = millis();
    unsigned long leftTimerTemp = leftTimer;
    unsigned long rightTimerTemp = rightTimer;
    float leftDistanceTemp = leftDistance;
    float rightDistanceTemp = rightDistance;
    //Serial.print("WTF: ");
    //Serial.print((leftDistanceTemp - leftDistanceSaved)/((timer-leftTimerTemp)*0.001));
    //TIMES NEED TO BE IN SECONDS!!
    //Serial.print(" Right Distance Saved: ");
    //Serial.print(rightDistanceSaved);
    //Serial.print(" ");
    VL = (leftDistanceTemp - leftDistanceSaved)/((timer-leftTimerTemp)*0.001)/180*PI;
    VR = (rightDistanceTemp - rightDistanceSaved)/((timer-rightTimerTemp)*0.001)/180*PI;
    //Serial.print(" VR: ");
    //Serial.print(VR);
    //Serial.print(" VL: ");
    //Serial.print(VL);
    //Serial.print(" ");
    VL = VL*WHEELRADIUS;
    VR = VR*WHEELRADIUS; //linear velocity in m/s

    float linearVelocity = (VR + VL)/2.0;
    float angularVelocity = (VR - VL)/WIDTH;

    float t = timer - integrationTimer;//time since integration loop was last run. 
    t = t/1000.0;
    
    float k00 = linearVelocity * cos(aj.heading);
    float k01 = linearVelocity * sin(aj.heading);
    float k02 = angularVelocity;

    float k10 = linearVelocity * cos(aj.heading + t/2*k02);
    float k11 = linearVelocity * sin(aj.heading + t/2*k02);
    float k12 = angularVelocity;

    float k20 = linearVelocity * cos(aj.heading + t/2*k12);
    float k21 = linearVelocity * sin(aj.heading + t/2*k12);
    float k22 = angularVelocity;

<<<<<<< Updated upstream
  void countLEncoder(){ // interrupt function for left encoder
  if(millis() - leftTimer > 5){
      leftDistance = leftDistance + ARCPERTICK;
=======
    float k30 = linearVelocity * cos(aj.heading + t*k22);
    float k31 = linearVelocity * sin(aj.heading + t*k22);
    float k32 = angularVelocity; 

    aj.x = aj.x + t/6*(k00 + 2*(k10+k20)+k30);
    aj.y = aj.y + t/6*(k01 + 2*(k11+k21) + k31);
    aj.heading = aj.heading + t/6*(k02 + 2*(k12+k22)+k32);

    //Serial.print("Time: ");
    //Serial.print(timer);
    //Serial.print(", Linear Velocity: ");
    //Serial.print(linearVelocity);
    //Serial.print(" , Angular Velocity: ");
    //Serial.println(angularVelocity);

    integrationTimer = timer;
    leftDistanceSaved = leftDistanceTemp;
    rightDistanceSaved = rightDistanceTemp;
    
    //take care of the overflow
    //if(abs(aj.heading)>PI){
     // if(aj.heading < 0){
    //    aj.heading = 2*PI + aj.heading;
     // }else if(aj.heading > 0){
    //    aj.heading = -2*PI + aj.heading;
     // }
    //}
>>>>>>> Stashed changes
  }
  
void countLEncoder(){ // interrupt function for left encoder
  
  leftDistance = leftDistance + ARCPERTICK;
  leftTimer = millis();
}

void countREncoder(){ // interrupt function for right encoder
<<<<<<< Updated upstream
  if(millis() - rightTimer > 5){
      rightDistance = rightDistance + ARCPERTICK;
  }
=======
  rightDistance = rightDistance + ARCPERTICK;
>>>>>>> Stashed changes
  rightTimer = millis();
}
