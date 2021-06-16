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
   //determine the path that connects the leftmost part of both obstacles.
   //ideally we want to hit the front and back left side of both obstacles. 
   float * arrx1 = ObstacleOneX.ToArray();
   float * arry1 = ObstacleOneY.ToArray();
   float * arrx2 = ObstacleTwoX.ToArray();
    float * arry2 = ObstacleTwoY.ToArray();

   float x1 = 10000;
   float x2 = 10000;
    float y1; //want largest value
    float y2; //want smallest value

    int index = 0;
   
   //sort through first arrays to find the smallest x value
   Serial.println(" ");
   Serial.println(" ");
   if(obstaclesSeen > 0){
    
    for(int i = 0; i < ObstacleOneX.GetSize(); i++){
      if(arrx1[i]<x1){
        x1 = arrx1[i];
        y1 = arry1[i];
      }
    }

    if(obstaclesSeen > 1){

    
      
    for(int i = 0; i < ObstacleTwoX.GetSize(); i++){
      if(arrx2[i]<x2){
        x2 = arrx2[i];
        y2 = arry2[i];
      }
    } 
    
   }
   
    
   }


   //??? Error is down here sommewhere
   if(obstaclesSeen > 1) {
    //append the fist set of points to the route back.
    PathBack[index][0] = x2 + PATHMARGIN*1.5;
    PathBack[index][1] = y2;

    index++; 
    PathBackLength++;

    //Serial.println("Second Obstacle Added to Path");
   }
   if(obstaclesSeen > 0){
    PathBack[index][0] = x1 + PATHMARGIN*3;
    PathBack[index][1] = y1;

    index++; 
    PathBackLength++;
    //Serial.println("First Obstacle Added to Path");
   }
   
   PathBack[index][0] = PATHMARGIN*4.5;
   PathBack[index][1] = 0;

    //Serial.println("Origin Added to Path");
   
   PathBackLength++;
   
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
      Serial.println(arry2[i]);
    }
  }
}
//MAKE THIS NON BLOCKING
void NavigateObstacle(int obstacle){
  
  ReadRightIR(); //theoretically we should only need the right ir for this.
  bool pastIRvalue = RightAverage;
  //loop that goes forwards a bit, then turns in the direction indicated by sensor.
  RecordPosition(obstacle);
  unsigned long recordTimer = millis();
  unsigned long startTime = recordTimer;
  unsigned long StraightTime = millis();
  bool flag = false;
  do{
    ReadRightIR();
    //record position on time interval.
    if(millis()-recordTimer > 250){
      //Serial.println("Recording position");
      RecordPosition(obstacle);
      recordTimer = millis();
    }
    
      if(RightAverage==LOW){ //on
        flag = false;
        Serial.println("Turn Left");
        left_motor.setSpeed(60);
        right_motor.setSpeed(255);
        //flag = false;
      }else{ //off
        if(pastIRvalue == LOW){
          flag = true;
          StraightTime = millis();
        }

        if(flag == true){
          if(millis() - StraightTime > 300){
            flag = false;
            Serial.println("Broke Free");
          }
          left_motor.setSpeed(170);
          right_motor.setSpeed(170);
          Serial.println("Going Straight");
        } else if(flag == false){
          Serial.println("Turning Right");
          left_motor.setSpeed(255);
          right_motor.setSpeed(60);
        }
      }    
    pastIRvalue = RightAverage;
  }while(aj.heading > -1.10 );
  obstaclesSeen++;
  obstacleTimer = millis();
  
}
