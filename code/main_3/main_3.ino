#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif
#define KP_NUM 10
#define KP_DEN 1
#define KI_NUM 1
#define KI_DEN 5
#define KD_NUM 0
#define KD_DEN 50
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

unsigned long tPrev;
double e_roll = 0; // error now
double ePrev_roll = 0; // previous error
double eInt_roll = 0;// integrated error 
double eDiff_roll = 0;// differentiated error

double e_pitch = 0; // error now
double ePrev_pitch = 0; // previous error
double eInt_pitch = 0;// integrated error 
double eDiff_pitch = 0;// differentiated error

double x = 0; //state 
double xPrev = 0; //previous state
double xDiff = 0;// differetiated state
double xRef = 0;//ref stae 

double y = 0; //state 
double yPrev = 0; //previous state
double yDiff = 0;// differetiated state
double yRef = 0;//ref stae

double z = 0; //state 
double zPrev = 0; //previous state
double zDiff = 0;// differetiated state
double zRef = 0;//ref stae

double u1 = 0;
double u2 = 0;// input 
int n = 1;


#include <ESP32Servo.h> 

Servo myservo1;  // create servo object to control a servo
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo edf;

// Possible PWM GPIO pins on the ESP32: 0(used by on-board button),2,4,5(used by on-board LED),12-19,21-23,25-27,32-33 
int servoPin1 = 18; 
int servoPin2 = 23;
int servoPin3 = 19;
int servoPin4 = 32;
int edfPin    = 33;// GPIO pin used to connect the servo control (digital out)
// Possible ADC pins on the ESP32: 0,2,4,12-15,32-39; 34-39 are recommended for analog input
int potPin = 20;        // GPIO pin used to connect the potentiometer (analog in)
int ADC_Max = 4096;     // This is the default ADC max value on the ESP32 (12 bit ADC width);
                        // this width can be set (in low-level oode) from 9-12 bits, for a
                        // a range of max values of 512-4096
  
int val;    // variable to read the value from the analog pin


MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, apitch, aroll, apitch_accel, aroll_accel, apitch_gyro = 0, aroll_gyro = 0, dr = 0, dp = 0, dy = 0, roll = 0, pitch = 0, yaw = 0, dt = 0.020;
float gX, gY, gZ ,gXave = -2.44, gYave = -1.56, gZave = 0.46, mDirection, mX, mY, mZ;
float K1 = 0.10, K2 = 0.90;
float ref_Roll = 0, ref_Pitch = 0;
int s_1 , s_2 , s_3 , s_4 , s_edf = 70;
void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;

  //servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo1.setPeriodHertz(50);// Standard 50hz servo
  myservo2.setPeriodHertz(50);
  myservo3.setPeriodHertz(50);
  myservo4.setPeriodHertz(50);
  edf.setPeriodHertz(50);
  myservo1.attach(servoPin1, 500, 2400); 
  myservo2.attach(servoPin2, 500, 2400);
  myservo3.attach(servoPin3, 500, 2400);
  myservo4.attach(servoPin4, 500, 2400);
  edf.attach(edfPin, 1000, 2000);
}

void loop() {
  uint8_t sensorId;
  if (mySensor.readId(&sensorId) == 0) {
    //Serial.print("sensorId: " + String(sensorId));
  } else {
    //Serial.println("Cannot read sensorId");
  }

  if (mySensor.accelUpdate() == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    apitch_accel = asin((float)aZ/aSqrt)*57.296;
    aroll_accel = asin((float)aX/aSqrt)*-57.296;// 57.296 = 1/(3.142/180)
    
    //Serial.print(" accelX: " + String(aX));
    //Serial.print("accelY: " + String(aY));
    //Serial.print("accelZ: " + String(aZ));
    //Serial.print("accelSqrt: " + String(aSqrt));
    Serial.print("apitch: " + String(apitch_accel));Serial.print("\t");
    Serial.print("aroll: " + String(aroll_accel));Serial.print("\t");
  } else {
    Serial.println("Cannod read accel values");
  }
  
  if (mySensor.gyroUpdate() == 0) {
    gX = mySensor.gyroX();// - gXave;
    gY = mySensor.gyroY();// - gYave;
    gZ = mySensor.gyroZ();// - gZave;
    
    Serial.print("gyroX: " + String(gX)); Serial.print("\t");       
    Serial.print("gyroY: " + String(gY));Serial.print("\t");
    Serial.print("gyroZ: " + String(gZ));Serial.print("\t");
  } else {
    Serial.println("Cannot read gyro values");
  }
  n++;

  x += gX*dt;
  Serial.print("X angle:");
  Serial.print(x);Serial.print("\t");
  y += gY*dt;
  Serial.print("Y angle:");
  Serial.print(y);Serial.print("\t");
  z += gZ*dt;
  Serial.print("Z angle:");
  Serial.print(z);Serial.print("\t");
  apitch_gyro -= gX*dt;
  aroll_gyro   -= gZ*dt;
  
  aroll_gyro -= apitch_gyro*sin(gY*dt*(3.142/180));
  apitch_gyro += aroll_gyro*sin(gY*dt*(3.142/180));


  Serial.print("roll gyro:" + String(aroll_gyro));Serial.print("\t");
  Serial.print("pitch gyro:" + String(apitch_gyro));Serial.print("\t");

  roll += dr*dt;
  pitch += dp*dt;
  yaw += dy*dt;

  apitch = apitch_accel*K1 + (apitch -gX*dt- apitch_gyro*sin(gY*dt*(3.142/180)))*K2;
  aroll = aroll_accel*K1 + (aroll -gZ*dt + aroll_gyro*sin(gY*dt*(3.142/180)))*K2;
  
  Serial.print("roll:" + String(aroll));Serial.print("\t");
  Serial.print("pitch:" + String(apitch));Serial.print("\t");
  u1 = roll_PID(ref_Roll, aroll);
  Serial.print("uroll:" + String(u1));Serial.print("\t");
  u2 = pitch_PID(ref_Pitch, apitch);
  Serial.print("uroll:" + String(u2));Serial.print("\t");
  //output
  s_1 = (map(-u2, -127, 128, 65, 115) + map(-u1, -127, 128, 65, 115))/2;
  s_2 = (map(-u2, -127, 128, 65, 115) + map(u1, -127, 128, 65, 115))/2;
  s_3 = (map(u2, -127, 128, 65, 115) + map(u1, -127, 128, 65, 115))/2;
  s_4 = (map(u2, -127, 128, 65, 115) + map(-u1, -127, 128, 65, 115))/2;
  

  s_1 = constrain(s_1,65,115);
  s_2 = constrain(s_2,65,115);
  s_3 = constrain(s_3,65,115);
  s_4 = constrain(s_4,65,115);
  Serial.print("s_1:" + String(s_1));Serial.print("\t");
  Serial.print("s_2:" + String(s_2));Serial.print("\t");
  Serial.print("s_3:" + String(s_3));Serial.print("\t");
  Serial.print("s_4:" + String(s_4));Serial.print("\t");
  Serial.print("s_edf:" + String(s_edf));Serial.print("\t");
  myservo1.write(s_1); 
  myservo2.write(s_2);
  myservo3.write(s_3);
  myservo4.write(s_4);
  /*
  if (s_edf < 60 && s_edf >=0 ){
    s_edf = s_edf + 1;
  }else{
    s_edf + s_edf - 1;
  }
  */
  
  if(apitch > 45 || aroll > 45){
    edf.write(0);
  }else{
    edf.write(s_edf);
  }
 
  
  
  unsigned int tProc = millis() - tPrev;
  Serial.print("tProc:");
  Serial.print(tProc);
  if(tProc < 20){
    delay(20 - tProc);
  } 
  tPrev = millis();
  //Serial.print(" at " + String(millis()) + "ms");
  Serial.println(""); // Add an empty line
}
