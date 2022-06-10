//PID
#include <PID_v1.h>

// IMU
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include <Servo.h>

Servo myservo;

// IMU variables

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define Gyr_Gain 0.00763358 

float AccelX;
float AccelY;
float AccelZ;

float GyroX;
float GyroY;
float GyroZ;

float mixX;
float mixY;

float pitchAccel, rollAccel;

float IMUpitch;
float IMUroll;

float output1;
float output2;

float pot;

// PID

double Pk1 = 10; 
double Ik1 = 50;
double Dk1 = 0.01;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
unsigned long count;


// ****************** SETUP ******************************

void setup() {

  pinMode(A9, INPUT);

  // initialize serial communication
  Serial.begin(115200);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-600, 600);
  PID1.SetSampleTime(10);

  Wire.begin();   
  accelgyro.initialize();

  myservo.attach(0);

}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {  // start timed event

    previousMillis = currentMillis;

    pot = analogRead(A9);
    pot = pot/975;

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
    AccelX = ax;
    AccelY = ay;
    AccelZ = az;
    GyroX = Gyr_Gain * (gx);
    GyroY = Gyr_Gain * (gy)*-1;
    GyroZ = Gyr_Gain * (gz);
  
    AccelY = (atan2(AccelY, AccelZ) * 180 / PI);
    AccelX = (atan2(AccelX, AccelZ) * 180 / PI);

    float dt = 0.01;
    float K = 0.9;
    float A = K / (K + dt);
  
    mixX = A *(mixX+GyroX*dt) + (1-A)*AccelY;    
    mixY = A *(mixY+GyroY*dt) + (1-A)*AccelX; 

    IMUroll = mixX + 5;

    Input1 = IMUroll;
    Setpoint1 = 0;    
    PID1.Compute();

    Output1 = Output1 * pot;

    output1 = Output1 + 1500;
    output1 = constrain(output1,1000,2000);

    if (output1 < 1500) {         // remove VESC deadband
      output2 = output1 - 80;
    }

    else if (output1 > 1500) {
      output2 = output1 + 70;
    }

    Serial.print(pot);
    Serial.print(" , ");
    Serial.println(output2);

    if (pot == 0) {
        myservo.writeMicroseconds(1500);
    }

    else {
        myservo.writeMicroseconds(output2);
    }


   }     // end of timed loop

}       // end  of main loop








