//ODrive
#include <ODriveArduino.h>

//PID
#include <PID_v1.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}

//ODrive Objects
ODriveArduino odrive1(Serial3);

int pot;
int but1;
int sw1;
int sw2;

float velocity;
float trimPot;

float accum;
float accumTrimmed;

int requested_state = 0;

float var1;       // data received from IMU
float var2;
float IMUroll;
float IMUpitch;
int check;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;         // time constant for timer
  
double Pk1 = 1.7;           // balancing PID values
double Ik1 = 21;
double Dk1 = 0.009;

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - motor position

void setup() {

  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  pinMode(A0, INPUT);
  pinMode(47, INPUT_PULLUP);
  pinMode(49, INPUT_PULLUP);
  pinMode(51, INPUT_PULLUP);
  pinMode(53, INPUT_PULLUP);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-100, 100);
  PID1.SetSampleTime(10);
}

void loop() {

    currentMillis = millis();
    if (currentMillis - previousMillis >= 10) {  // start timed event

      previousMillis = currentMillis;

      pot = analogRead(A0);
      but1 = digitalRead(47);
      sw1 = digitalRead(51);
      sw2 = digitalRead(53);
      
      trimPot = (float) (pot - 512) / 100;    // read trim pit and scale to degrees

      if (Serial2.available() > 1){           // receive IMU data from serial IMU
        check = Serial2.parseInt();
        if (check == 500) {                   // look for check character to check it's the start of the data
            var1 = Serial2.parseInt();
            var2 = Serial2.parseInt();

            if (Serial2.read() == '\n') {     // end of IMU data 
                IMUpitch = var1 / 100;         // divide by 100 to get our decimal places back
                IMUroll = var2 / 100;
            }
        }
      }        

      Input1 = IMUroll;                 // PID input is IMU data
      Setpoint1 = trimPot - accum;      // PID setpoint is trim pot minus the observation controller output

      accum = accum - (Output1/150);    // observation controller
      accum = (constrain(accum,-1,1));  // constrain the data

      Serial.println(accum);            // debug      
      
      PID1.Compute();                   // compute PID

      velocity = Output1;      
    
      if (but1 == 0) {                  // init ODrive
          OdriveInit1();
      }
    
      if (sw1 == 1) {                 // disable motor
        odrive1.SetVelocity(0, 0);
      }
    
      else {                          // drive motor
        odrive1.SetVelocity(0, velocity);
      }
    

    }   // end of timed loop

}
