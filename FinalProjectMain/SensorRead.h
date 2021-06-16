
void ReadUltrasonic() {
  // subtract the last reading:
  UltrasonicTotal = UltrasonicTotal - UltrasonicReadings[UltrasonicReadIndex];
  // read from the sensor:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  UltrasonicReadings[UltrasonicReadIndex] = duration * 0.034 / 2;;
  // add the reading to the total:
  UltrasonicTotal = UltrasonicTotal + UltrasonicReadings[UltrasonicReadIndex];
  // advance to the next position in the array:
  UltrasonicReadIndex = UltrasonicReadIndex + 1;

  // if we're at the end of the array...
  if (UltrasonicReadIndex >= numReadings) {
    // ...wrap around to the beginning:
    UltrasonicReadIndex = 0;
  }

  // calculate the average:
  UltrasonicAverage = UltrasonicTotal / numReadings;
  // send it to the computer as ASCII digits
  //Serial.print("Ultrasonic Average ");
  //Serial.println(UltrasonicAverage);
  delay(1);
}

void ReadBottomIR() {
  // subtract the last reading:
  BottomTotal = BottomTotal - BottomReadings[BottomReadIndex];
  // read from the sensor:
  BottomReadings[BottomReadIndex] = analogRead(irPin);
  // add the reading to the total:
  BottomTotal = BottomTotal + BottomReadings[BottomReadIndex];
  // advance to the next position in the array:
  BottomReadIndex = BottomReadIndex + 1;

  // if we're at the end of the array...
  if (BottomReadIndex >= numReadings) {
    // ...wrap around to the beginning:
    BottomReadIndex = 0;
  }

  // calculate the average:
  BottomAverage = BottomTotal / numReadings;
  // send it to the computer as ASCII digits
  //Serial.print("Bottom Average ");
  //Serial.println(BottomAverage);
  delay(1);
}

void ReadLeftIR(){
  LeftAverage = digitalRead(leftProxPin);
}

void ReadRightIR(){
  RightAverage = digitalRead(rightProxPin);
}

void CollectSensorData() {
  ReadUltrasonic();
  ReadBottomIR();
  ReadLeftIR();
  ReadRightIR();
  //Serial.print("Ultrasonic: ");
  //Serial.print(UltrasonicAverage);
  //Serial.print(",Bottom Ir: ");
  //Serial.print(BottomAverage);
  //Serial.print(",Left Ir: ");
  //Serial.print(LeftAverage);
  //Serial.print(",Right Ir: ");
  //Serial.println(RightAverage);
}
