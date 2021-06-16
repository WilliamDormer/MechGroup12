/*
void FindTarget(){

  //start by doing circles, 
  //create a series of position pairs that represent a spiral pattern, ideally where the change in radius from one rotation to another is the width of the ir sensor measurement area
  //origin will be at current point

  Position SavedLocation;
  SavedLocation = aj; //stores position of robot before search algorithm.
  const float searchRadius = 50.0;
  const float radiusIncreasePerTurn = 5;
  const int numIntervals = 8;
  const float increasePerInterval = radiusIncreasePerTurn / numIntervals;

  const int arraySize = (int)((searchRadius/increasePerInterval)+0.5);

  //Serial.println(arraySize);
  
  float positionArray[arraySize][2]; //arraySize spots, 2 locations, one for x and one for y

  
  float angle = SavedLocation.heading;
  float currentRadius = 0;

  for(int i = 0; i < arraySize; i++){

    //hypotenuse is increasePerInterval
    //each increase is pi / 8 radians

    angle += PI/4.0;
    currentRadius += increasePerInterval;

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
    //Serial.println(positionArray[i][1]);
 }
 
 Serial.println("Done Allocating space");
    
    bool flag = false;
    for(int i = 4; i < arraySize; i++){
      Serial.print("i: ");
      Serial.print(i);
      //delay(2000);
      float nextPoint = distanceToTarget(positionArray[i][0], positionArray[i][1]);
      while(nextPoint > 8.0 && flag == false){
        ReadBottomIR();
        nextPoint = distanceToTarget(positionArray[i][0], positionArray[i][1]);
        Serial.print(",NextPoint: ");
        Serial.println(nextPoint);
        TravelToDestination(positionArray[i][0], positionArray[i][1]);
          if(setPointBottomPerm - BottomAverage > IRTHRESHOLD || setPointBottomPerm - BottomAverage < -IRTHRESHOLD){
            flag = true;
          }
      }
      delay(1000);
      if(flag == true){
        Serial.println("Target Found");
        break;
      }
    }  
  //then generate an array of that size. 
  
  
  //gradually increase the target radius and the target angle on each iteration to plot a spiral// max radius is 1m
  //then execute until the sensor detects the final location. 
}
*/
void FindTarget(){
  //new strategy, just spin and slowly increase inner motor speed.

  //first need to rotate the vehicle left a bit or else the sprial grows to the right, but does not actually scan the left hemisphere.
  right_motor.setSpeed(255);
  left_motor.setSpeed(10);
  delay(500);
  
  left_motor.setSpeed(255);
  int i = 100;
  unsigned long timer = millis();
  while(true){
    right_motor.setSpeed(i);
    if(millis()-timer > 400){
      i++;
      Serial.println(i);
      timer = millis();
    }
    ReadBottomIR();
    if(setPointBottomPerm - BottomAverage > IRTHRESHOLD || setPointBottomPerm - BottomAverage < -IRTHRESHOLD){
      break;
    }
  }
  
  
}
