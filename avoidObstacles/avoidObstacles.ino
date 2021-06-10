// The IMU odometry library is provided by Gregory Kelly, modified by William Dormer
// Algorithms are drawn from MIT OpenCourseware: https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-186-mobile-autonomous-systems-laboratory-january-iap-2005/study-materials/odomtutorial.pdf
// It is distributed as-is with obfuscation, in accordance with the course-specific academic entegrity policy.

//#include <DeadReckoner.h>
//#include "Odom.h"

//smoothing code for arduino car

//relevant libraries
#include <AFMotor.h>
#include <PID_v2.h>

//pin definitions for sensors
#define echoPin A4
#define trigPin A3
#define irPin A5
#define leftProxPin A2
#define rightProxPin A1
#define LEncoderPin 3
#define REncoderPin 2

#define TICKSPERROT 20.0
#define CIRCUMFERENCE 22.0 //cm
#define ARCPERTICK 0.55//1.1 //cm
#define IRTHRESHOLD 25.0

//required for deadreckoning library

// MEASUREMENTS
// The units for all measurements must be consistent. 
// You can use any length unit as desired.
#define RADIUS 35.0 // wheel radius in mm
#define LENGTH 130.0 // wheel base length in mm
#define WIDTH 13.0 //wheel to wheel width in cm

// TIME INTERVALS
//unsigned long POSITION_COMPUTE_INTERVAL = 50; // milliseconds
//#define SEND_INTERVAL 100 // milliseconds


//state defintions
int state = 0; 
//0 is the starting state
//1 is moving towards the destination using odometry
//2 is avoid the obstacles 
//3 is destination reached.
bool stateChange = true;

// Previous times for computing elapsed time.
//unsigned long prevPositionComputeTime = 0, prevSendTime = 0;

unsigned long leftTimer;
unsigned long rightTimer;

// Previous x and y coordinate.
double prevX = 0, prevY = 0;

//variables for sensors
const int numReadings = 10;

//for the ultrasonic
volatile int UltrasonicReadings[numReadings];
volatile int UltrasonicReadIndex = 0;
volatile int UltrasonicTotal = 0;
volatile int UltrasonicAverage = 0;

//for bottom IR
int BottomReadings[numReadings];
int BottomReadIndex = 0;
int BottomTotal = 0;
int BottomAverage = 0;

//for left IR
int LeftReadings[numReadings];
int LeftReadIndex = 0;
float LeftTotal = 0;
bool LeftAverage = false;

//for right IR
int RightReadings[numReadings];
int RightReadIndex = 0;
float RightTotal = 0;
bool RightAverage = false;

//setup the dc motor controller
AF_DCMotor left_motor(1, MOTOR34_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(3, MOTOR12_1KHZ); // right motor to M3 on motor control board

//PID controller settings
double Kp = 0.3, Ki = 0.4, Kd = 0.2;
double EncDelta=0;
double OutDelta=-20.0;
double EncDeltaSet=0;
int LPWM=255;
int RPWM=255;

PID DiffSpeed(&EncDelta, &OutDelta, &EncDeltaSet,Kp,Ki,Kd, DIRECT);  // set the PID controller, which will try to make EncDelta 
                                                                     // be the same as by EncDeltaSet (i.e. zero) by setting a value
                                                                     // OutDelta, which will be used to adjust the motor speed settings.
//encoder count information                                                                     // This will be updated every 200 ms by default
volatile unsigned int LeftEncoderCount = 0;
volatile unsigned int RightEncoderCount = 0;

//just for testing gregs code
float leftDistance = 0;
float rightDistance = 0;
struct Position {
    float x;
    float y;
    float heading;
  };
Position aj;
  float*aa;
  float*ab;
  float ac;
  float ad=0;
  float ae=0;

//necessary flags
bool destinationReached = false;

//storage for sensor values
volatile int setPointBottom;
volatile int setPointBottomPerm;
volatile int setPointUltrasonic;

//call the library
//DeadReckoner(&LeftEncoderCount, &RightEncoderCount, POSITION_COMPUTE_INTERVAL, TICKSPERROT, RADIUS, LENGTH);
//DeadReckoner deadReckoner(&LeftEncoderCount, &RightEncoderCount, TICKSPERROT, RADIUS, LENGTH);
//DeadReckoner deadReckoner = deadReckoner.setParams(&LeftEncoderCount,&RightEncoderCount, unsigned long 50, double 10.0, double 12.0, double 14.0);
void setup() {
  //attach interupts for motor encoders 
  attachInterrupt(1, countLEncoder, RISING); //calls on any rising level on pin 3 (interrupt #1, soldered to Pin3) 
  attachInterrupt(0, countREncoder, RISING); //calls on any rising level on pin 2 (interrupt #0, connected to header on Pin2) 
  interrupts();
  
  //pin assignments
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(irPin, INPUT);
  pinMode(leftProxPin, INPUT);
  pinMode(rightProxPin, INPUT);

  Serial.begin(115200);
  Serial.println("Starting Operation");

  leftTimer = millis();
  rightTimer = millis();

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    BottomReadings[thisReading] = 0;
    UltrasonicReadings[thisReading] = 0;
    LeftReadings[thisReading] = 0;
  }

  // turn motors off initially
  left_motor.run(RELEASE);  
  right_motor.run(RELEASE);

  //initialization for PID controller
  //  DiffSpeed.SetMode(MANUAL);  // used to disable the PID controller so you can set motor values manually
  DiffSpeed.SetMode(AUTOMATIC);
  DiffSpeed.SetOutputLimits(-150,150);
//   DiffSpeed.SetSampleTime(200);  // By default the PID is updated every 200 ms - if you need to change it you use this function

  //self sensor test
  delay(100);
  sensorTest();
  Serial.println("Sensor Test finished. Starting operation in 1 second");
  delay(1000);
  LeftEncoderCount = 0;
  RightEncoderCount = 0;

  //this is where the odometry stuff needs to go
  leftDistance = 0;
  rightDistance = 0;
  init(&leftDistance,&rightDistance,WIDTH);
}

void loop() {
  // put your main code here, to run repeatedly:
  //CollectSensorData();
  //toPlot();
  if(state == 0){
    //Serial.println("Beginning operation...");
    //engage motors
    left_motor.run(FORWARD);  
    right_motor.run(FORWARD);
    left_motor.setSpeed(100);
    right_motor.setSpeed(100);
    state++;
  }else if(state == 1){
    //Serial.println("Navigating using Odometry...");
    //stateChange = false;
    
    CollectSensorData();
    DetectObstacle();
    volatile float angle = headingTo(0,90);//get the angle to the destination, 100 cm from the starting location
    //angle = angle;
    if(abs(abs(angle) - abs(aj.heading)) < abs(0.15)){ //if they are within a margin of error.
      left_motor.setSpeed(150);
      right_motor.setSpeed(160);
    }else if(angle < aj.heading){ //need to turn right i think
      Serial.println("Left Turn");
      left_motor.setSpeed(30);
      right_motor.setSpeed(180);
      //turn right and obstacle detection
    }else if(angle > aj.heading){ //need to turn left i think
      //turn left and obstacle detection
      Serial.println("Right Turn");
      left_motor.setSpeed(180);
      right_motor.setSpeed(30);
    }
    Serial.print("Heading Straight: Current Angle: ");
      Serial.print(aj.heading);
      Serial.print(" Target Angle: ");
      Serial.println(angle);
  }else if(state == 2){
    //Serial.println("Navigating around Obstacle...");
    CollectSensorData();
    Serial.print("Ultrasonic value ");
    Serial.println(UltrasonicAverage);
    if(LeftAverage == 1){
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
    } else if(UltrasonicAverage < 15){
      left_motor.setSpeed(30);
      right_motor.setSpeed(180);
    } else if(RightAverage == 1){
      left_motor.setSpeed(30);
      right_motor.setSpeed(180);
    }
    //if the ultrasonic sensor sees something, rotate left
    //if the left sensor senses something, pause
    //if the right one senses something, rotate sharp left
    //if none of them sense something, then go back to state 1 
    DetectObstacle();
  }else if(state == 3){
    //Serial.println("Destination Reached");
    left_motor.run(RELEASE);  
    right_motor.run(RELEASE);
  }
}

void DetectObstacle(){
  if(UltrasonicAverage < 15 || LeftAverage == 1 || RightAverage == 1){
    state = 2;
    //stateChange = true;
  }else if (BottomAverage-setPointBottomPerm > IRTHRESHOLD || BottomAverage-setPointBottomPerm < -IRTHRESHOLD){
    state = 3;
    //stateChange = true;
  }else{
    delay(5); //for debug
    state = 1;
  }
}

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
  float headingTo(float x, float y) {
    return atan2((x-aj.x)*2, y-aj.y);
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

void Forward(){
  // Calculate the difference in encoder counts on each wheel, convert to a double to use with the PID

  EncDelta = (double)(1.0*LeftEncoderCount - 1.0*RightEncoderCount); // calculate the difference in encoder values
  DiffSpeed.Compute(); // compute the value of OutDelta using the PID controller
  Serial.print("OutDelta: ");
  Serial.println(OutDelta);
  LPWM = (int)round(130+OutDelta);
  RPWM = (int)round(130-OutDelta);

  // insert other code as necessary
  
  left_motor.setSpeed(LPWM); // adjust speed values applied to each motor using the updates from the PID controller
  right_motor.setSpeed(RPWM); 
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
  
  LeftTotal = LeftTotal - LeftReadings[LeftReadIndex];
  bool temp = digitalRead(leftProxPin);
  temp = !temp;
  //Serial.println(temp);
  if(temp == 0){
    LeftReadings[LeftReadIndex] = 0;
  } else {
    LeftReadings[LeftReadIndex] = 1;
  }
  //Serial.println(LeftReadings[LeftReadIndex]);
  //Serial.println(LeftTotal);
  LeftTotal = LeftTotal + LeftReadings[LeftReadIndex];
  //Serial.println(LeftTotal);
  LeftReadIndex = LeftReadIndex + 1;

  if (LeftReadIndex >= numReadings) {
    // ...wrap around to the beginning:
    LeftReadIndex = 0;
  }
  double avg = LeftTotal / numReadings;
  //Serial.println(avg);
  if(avg <0.500){
    LeftAverage = LOW;
    //Serial.println("Left Sensor OFF");
  }else{
    LeftAverage = HIGH;
    //Serial.println("Left Sensor ON");
  }
  delay(1);
  
}

void ReadRightIR(){
  
  RightTotal = RightTotal - RightReadings[RightReadIndex];
  bool temp = digitalRead(rightProxPin);
  temp = !temp;
  //Serial.println(temp);
  if(temp == 0){
    RightReadings[RightReadIndex] = 0;
  } else {
    RightReadings[RightReadIndex] = 1;
  }
  //Serial.println(RightReadings[RightReadIndex]);
  //Serial.println(RightTotal);
  RightTotal = RightTotal + RightReadings[RightReadIndex];
  //Serial.println(RightTotal);
  RightReadIndex = RightReadIndex + 1;

  if (RightReadIndex >= numReadings) {
    // ...wrap around to the beginning:
    RightReadIndex = 0;
  }
  double avg = RightTotal / numReadings;
  //Serial.println(avg);
  if(avg <0.500){
    RightAverage = LOW;
    //Serial.println("Left Sensor OFF");
  }else{
    RightAverage = HIGH;
   // Serial.println("Left Sensor ON");
  }
  delay(1);
}

void sensorTest(){
  //sensor test
  boolean frontleft = LOW;
  boolean frontright = LOW;
  boolean bottom = LOW;
  
  boolean leftencoder = LOW;
  boolean rightencoder = LOW;
  boolean ultrasonic = LOW;
  
  boolean FlagReached = LOW;

  setPointBottom = 0;
  setPointUltrasonic = 0;

  for(int i = 0; i<25; i++){ //this is required to warm up the sensors for some reason, presumably so the averages get more accurate.
    CollectSensorData();
    delay(5);
  }
  
  while(!frontleft || !frontright || !bottom || !leftencoder || !rightencoder || !ultrasonic){
    if(bottom == LOW){ //this just saves memory resources
      ReadBottomIR();
      if(setPointBottom == 0){
        setPointBottom = BottomAverage;
        setPointBottomPerm = setPointBottom;
      } else {
        if(setPointBottom-BottomAverage >IRTHRESHOLD || setPointBottom-BottomAverage < -IRTHRESHOLD){
          bottom = HIGH;
          Serial.println("Bottom Sensor Attached");
        }
      }
    }
    if(frontleft == LOW){
      ReadLeftIR();
      if (LeftAverage == HIGH){
        Serial.println("Front Left Sensor Attached");
        frontleft = HIGH;
      }
    }
    if(frontright == LOW){
      ReadRightIR();
      if(RightAverage == HIGH){
        frontright = HIGH;
        Serial.println("Front Right Sensor Attached");
      }
    }
    if(leftencoder == LOW){
      if(LeftEncoderCount > 0){
        Serial.println("Left Encoder Sensor Attached");
        leftencoder = HIGH;
      }
    }
    if(rightencoder == LOW){
      if(RightEncoderCount > 0){
        Serial.println("Right Encoder Sensor Attached");
        rightencoder = HIGH;
      }
    }

    if(ultrasonic == LOW){
      ReadUltrasonic();
      if(setPointUltrasonic == 0){
        setPointUltrasonic = UltrasonicAverage;
      } else {
        if(setPointUltrasonic-UltrasonicAverage>10 || setPointUltrasonic-UltrasonicAverage < -10){
          ultrasonic = HIGH;
          Serial.println("UltraSonic Sensor Attached");
        }
      }
    }
  } //end of while loop.  
  Serial.println("Sensor Check Completed");
}

void countLEncoder(){ // interrupt function for left encoder
  if(millis() - leftTimer > 5){
      LeftEncoderCount++;
      leftDistance = leftDistance + ARCPERTICK;
  }
  leftTimer = millis();
      
}

void countREncoder(){ // interrupt function for right encoder
  if(millis() - rightTimer > 5){
      RightEncoderCount++;
      rightDistance = rightDistance + ARCPERTICK;
  }
  rightTimer = millis();
}
