#ifndef ODOM_H
#define ODOM_H

/**
 * The IMU odometry library is provided by Gregory Kelly.
 * Algorithms are drawn from MIT OpenCourseware: https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-186-mobile-autonomous-systems-laboratory-january-iap-2005/study-materials/odomtutorial.pdf
 * It is distributed as-is with obfuscation, in accordance with the course-specific academic entegrity policy.
 * It may be modified and redistributed with this disclaimer.
 * 
 * Provided functions
 *
 * Position struct has float x, float y, and float heading.
 * Units are cm, cm, radians
 *
 * void ODOM::init(float*, float*, float)
 * Initialise the IMU
 * This should be done after intialising the drivetrain and sensors
 * @see https://busylog.net/FILES2DW/doc8161.pdf page 134
 * @param leftDist Pointer to the distance for the left encoder (cm)
 * @param rightDist Pointer to the distance for the right encoder (cm)
 * @param width Width of the robot, distance between wheel centres (cm)
 *
 * Position ODOM::getPosition()
 * @return The robot's position
 *
 * float ODOM::headingTo()
 * Get the heading in radians to a target point
 * @param x x position of the point
 * @param y y position of the point
 * @return The heading in radians facing the point
 *
 * void ODOM::toPlot()
 * Prints out data for plotting with ArduPlot
*/

namespace Odom {
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
	void init(float*leftDist, float*rightDist, float width){
		noInterrupts(); //i guess this disables interupts for this part
		aj.x=0; //sets the x position to zero
		ac=width; //so ac represents the wheel to wheel width
		*ab=0; 
		aa=leftDist;
		ab=rightDist;
		TCCR1B=8|5;//no clue what this does
		*aa=0;TCCR1A=4;
		OCR1A=(int)(50 / 0.064);
		aj.heading=0; //sets the initial heading to zero
		TIMSK1=2;
		interrupts();
		aj.y=0; //sets the y position to zero
	}
	Position getPosition() {return aj;}
	void toPlot() {
		Serial.print(aj.x);
		Serial.print("\t");
		Serial.print(aj.y);
		Serial.print("\t");
		Serial.print(aj.heading*180/PI);
		Serial.println("");
	}
	float headingTo(float x, float y) {
		return atan2((x-aj.x)*2, y-aj.y);
	}
	ISR(TIMER1_COMPA_vect) {
		Serial.println("ISR");
		float af=*aa-ad;
		float ag=*ab-ae;
		ad=*aa;
		ae=*ab;
		float ah=(af+ag)/2.0;
		float ai=(ag-af)/ac;
		aj.x+=ah*sin(aj.heading);
		aj.y+=ah*cos(aj.heading);
		aj.heading+=ai;
	}
}

#endif
