#include <AFMotor.h>

AF_DCMotor left_motor(1, MOTOR34_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(3, MOTOR12_1KHZ); // right motor to M3 on motor control board

#define IRTHRESHOLD 25.0
#define ARCPERTICK 0.55//1.1 //cm
#define WIDTH 13.0 //wheel to wheel width in cm

#define DRIVESMOOTHFACTOR 300 //this is used to figure out how much the turning should correct based on angle.
#define BASEMOTORSPEED 170
#define SLOWDOWNFACTOR 1.2;

//odometry and motor encoding information
float leftDistance = 0;
float rightDistance = 0;

//timing information for encoders
unsigned long leftTimer;
unsigned long rightTimer;

struct Position { //odometry position information
    float x; //x offset from origin
    float y; //y offset from origin
    float heading; //angle in radians
  };
  
Position aj;
  float*aa;
  float*ab;
  float ac;
  float ad=0;
  float ae=0;

//Pin Allocations
#define echoPin A4
#define trigPin A3
#define irPin A5
#define leftProxPin A2
#define rightProxPin A1
#define LEncoderPin 3
#define REncoderPin 2

//Variables for sensors
const int numReadings = 10; //Determines the number of readings for the ultrasonic and bottom IR sensor smoothing. 

bool RightAverage = HIGH;
bool LeftAverage = HIGH;

//these variables allow for dynamic 
volatile int setPointBottom;
volatile int setPointBottomPerm;
volatile int setPointUltrasonic;

//Persistant Sensor information for the ultrasonic
volatile int UltrasonicReadings[numReadings];
volatile int UltrasonicReadIndex = 0;
volatile int UltrasonicTotal = 0;
volatile int UltrasonicAverage = 0;

//Persistant Sensor Information for bottom IR
int BottomReadings[numReadings];
int BottomReadIndex = 0;
int BottomTotal = 0;
int BottomAverage = 0;

#include "Odometry.h"
#include "SensorRead.h"
#include "SensorCheck.h"
#include "TravelToDestination.h"



#define SomeTest 3

void setup() {
  //attach interupts for motor encoders 
  attachInterrupt(1, countLEncoder, RISING); //calls on any rising level on pin 3 (interrupt #1, soldered to Pin3) 
  attachInterrupt(0, countREncoder, RISING); //calls on any rising level on pin 2 (interrupt #0, connected to header on Pin2) 
  interrupts();


  //setup the dc motor controller
  
  left_motor.run(RELEASE);  
  right_motor.run(RELEASE);

  //pin assignments
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(irPin, INPUT);
  pinMode(leftProxPin, INPUT);
  pinMode(rightProxPin, INPUT);
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting Serial Operation");

  //set reference for the motor encoder timers
  leftTimer = millis();
  rightTimer = millis();

  //initialize all the sensor arrays to a value
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    BottomReadings[thisReading] = 0; //has to be zero to not mess with future readings, result of which means that some time needs to be taken to calibrate them.
    UltrasonicReadings[thisReading] = 0;
  }

  for(int i = 0; i<numReadings; i++){ //this is required to warm up the sensors for some reason, presumably so the averages get more accurate.
    CollectSensorData();
    delay(5);
  }

  //init(&leftDistance,&rightDistance,WIDTH);
  //sensorTest();

  init(&leftDistance,&rightDistance,WIDTH);
  left_motor.run(FORWARD);
  right_motor.run(FORWARD);
}

void loop() {
 
  // put your main code here, to run repeatedly:
  //CollectSensorData();

  //plot out code that will drive the car in the correct direction based on the angle of the car.
  //first figure out if the angle to is working properly
  
  TravelToDestination(0.0,300.0);

  //toPlot();
  
}
