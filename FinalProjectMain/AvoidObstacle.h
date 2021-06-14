void RecordPosition(int obstacle){ //works
  if(obstacle == 0){ //use the first linked list pair
    ObstacleOneX.InsertTail(aj.x);
    ObstacleOneY.InsertTail(aj.y);
  }else if (obstacle == 1){ //use the second linked list pair
    ObstacleTwoX.InsertTail(aj.x);
    ObstacleTwoY.InsertTail(aj.y);
  }
}

float PickRoute(){
  //should return an array holding the coordinates that it should go.

  //if both obstacles are hit it is easy, we know it needs to go around each of them to the center.
  //if only one is there then we have a problem.
  //2 possible situations:
  //either it hit the first and missed the second:
  //or it hit the second and missed the first. 

  //one solution is to play the center, meaning that it only travels forwards along the center line, thus if it misses an obstacle, it is fine if it knows to return to the center path after clearing the other. 
}

void printLinkedList(int obstacle){ //works
  if(obstacle == 0){
    float * arrx1 = ObstacleOneX.ToArray();
    float * arry1 = ObstacleOneY.ToArray();
    Serial.println("First Obstacle: ");
    for(int i = 0; i<ObstacleOneX.GetSize(); i++){
      Serial.print("X: ");
      Serial.print(arrx1[i]);
      Serial.print("Y: ");
      Serial.println(arry1[i]);
    }
  }else if (obstacle == 1){
    float * arrx2 = ObstacleTwoX.ToArray();
    float * arry2 = ObstacleTwoY.ToArray();
    Serial.println("");
    Serial.println("Second Obstacle: ");
    for(int i = 0; i<ObstacleTwoX.GetSize(); i++){
      Serial.print("X: ");
      Serial.print(arrx2[i]);
      Serial.print("Y: ");
      Serial.print(arry2[i]);
    }
  }
}

void NavigateObstacle(int obstacle){
  //keep the right side of the vehicle facing the obstacle.
  //get close enough that the right IR sensor can detect the obstacle.
  //ReadUltrasonic();
  //ReadLeftIR();
  ReadRightIR(); //theoretically we should only need the right ir for this.
  bool pastIRvalue = RightAverage;
  //loop that goes forwards a bit, then turns in the direction indicated by sensor.
  RecordPosition(obstacle);
  float xStart;
  float yStart;
  if(obstacle == 0){ //use the first linked list pair
    xStart = ObstacleOneX.GetHead();
    yStart = ObstacleOneY.GetHead();
  }else if (obstacle == 1){ //use the second linked list pair
    xStart = ObstacleTwoX.GetHead();
    yStart = ObstacleTwoY.GetHead();
  }
  unsigned long recordTimer = millis();
  unsigned long startTime = recordTimer;
  do{
    ReadRightIR();
    //record position on time interval.
    if(millis()-recordTimer > 500){
      Serial.print("Recording position");
      RecordPosition(obstacle);
      recordTimer = millis();
    }
    
    if(RightAverage==LOW){ //on
      Serial.println("Turn Left");
      left_motor.setSpeed(0);
      right_motor.setSpeed(255);
    }else{ //off
      //on low to high, we want to check to make sure that we didn't pass the edge of the obstacle.
      if(pastIRvalue == LOW){
        Serial.println("Go Straight.");
        left_motor.setSpeed(170);
        right_motor.setSpeed(170);
        delay(200);
      }
      Serial.println("Turn Right");
      left_motor.setSpeed(180);
      right_motor.setSpeed(0);
    }
    pastIRvalue = RightAverage;
  }while(recordTimer - startTime < 2000 || distanceToTarget(xStart,yStart) > 10);

  //debug
  //Serial.println("While Ended");
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  delay(10000);
  //printLinkedList(0);
  
  //drive on that borderline, which should turn right when it is present, left when not.
  //store locations periodically,
  //when we reach a location that is within 5 cm of one of the other locations, then stop recording
  //at this point, determine which of the points was the closest to the goal.
  //then go to that point, by following the recorded points.
  //resume motion.
}
