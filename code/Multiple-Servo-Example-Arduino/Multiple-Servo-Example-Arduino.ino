/*
 * ESP32 Servo Example Using Arduino ESP32 Servo Library
 * John K. Bennett
 * March, 2017
 * 
 * This sketch uses the Arduino ESP32 Servo Library to sweep 4 servos in sequence.
 * 
 * Different servos require different pulse widths to vary servo angle, but the range is 
 * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
 * sweep 180 degrees, so the lowest number in the published range for a particular servo
 * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
 * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
 * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
 * degrees.
 * 
 * Circuit:
 * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
 * the ground wire is typically black or brown, and the signal wire is typically yellow,
 * orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
 * considerable power, we will connect servo power to the VBat pin of the ESP32 (located
 * near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS. 
 * 
 * We could also connect servo power to a separate external
 * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
 * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
 * connect to any available GPIO pins on the ESP32 (in this example, we use pins
 * 22, 19, 23, & 18).
 * 
 * In this example, we assume four Tower Pro SG90 small servos.
 * The published min and max for this servo are 500 and 2400, respectively.
 * These values actually drive the servos a little past 0 and 180, so
 * if you are particular, adjust the min and max values to match your needs.
 * Experimentally, 550 and 2350 are pretty close to 0 and 180.
 */

#include <ESP32Servo.h>

// create four servo objects 
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo edf;
// Published values for SG90 servos; adjust if needed
int minUs = 500;
int maxUs = 2400;

// These are all GPIO pins on the ESP32
// Recommended pins include 2,4,12-19,21-23,25-27,32-33 
int servo1Pin = 23;
int servo2Pin = 22;
int servo3Pin = 33;
int servo4Pin = 21;
int edfPin = 32;

int pos1 = 90; 
int pos2 = 90;
int pos3 = 90;
int pos4 = 90;// position in degrees
ESP32PWM pwm;
void setup() {
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	Serial.begin(115200);
	servo1.setPeriodHertz(50);      // Standard 50hz servo
	servo2.setPeriodHertz(50);      // Standard 50hz servo
	servo3.setPeriodHertz(50);      // Standard 50hz servo
	servo4.setPeriodHertz(50);      // Standard 50hz servo
	edf.setPeriodHertz(50);      // Standard 50hz servo


}

void loop() {
	servo1.attach(servo1Pin, minUs, maxUs);
	servo2.attach(servo2Pin, minUs, maxUs);
	//pwm.attachPin(27, 10000);//10khz
	servo3.attach(servo3Pin, minUs, maxUs);
	servo4.attach(servo4Pin, minUs, maxUs);
  edf.attach(edfPin, minUs, maxUs);


	servo1.write(pos1);
  servo2.write(pos2);
  servo3.write(pos3);
  servo4.write(pos4);
	delay(1);             // waits 20ms for the servo to reach the position

	servo1.detach();
	servo2.detach();
	servo3.detach();
	servo4.detach();
	//pwm.detachPin(27);

	delay(5000);

}
