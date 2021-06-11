

//Variables for sensors
volatile int setPointBottom;
volatile int setPointBottomPerm;
volatile int setPointUltrasonic;

#include "SensorCheck.h"

//Pin Allocations
#define echoPin A4
#define trigPin A3
#define irPin A5
#define leftProxPin A2
#define rightProxPin A1
#define LEncoderPin 3
#define REncoderPin 2

#define SomeTest 3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting Serial Operation");
  sensorTest();
}

void loop() {
  // put your main code here, to run repeatedly:

}
