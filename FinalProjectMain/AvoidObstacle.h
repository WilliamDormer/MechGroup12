void RecordPosition(int obstacle){
  if(obstacle == 0){ //use the first linked list pair
    ObstacleOneX.InsertTail(aj.x);
    ObstacleOneY.InsertTail(aj.y);
  }else if (obstacle == 1){ //use the second linked list pair
    ObstacleOneX.InsertTail(aj.x);
    ObstacleOneY.InsertTail(aj.y);
  }
}
