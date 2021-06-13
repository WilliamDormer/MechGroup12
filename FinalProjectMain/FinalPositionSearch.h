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
    Serial.println(positionArray[i][1]);
 }
 Serial.println("Done");

}
