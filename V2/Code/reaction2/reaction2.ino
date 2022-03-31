// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;  
    int16_t Select;    
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

float RLR = 0;
float RFB = 0;
float RT = 0;
float LLR = 0;
float LFB = 0;
float LT = 0;
int LTa = 0;

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
int pot2;
int but1;
int sw1;
int sw2;

float velocity;
float velocity2;
float trimPot;
float trimPot2;

float accum;
float accum2;

int requested_state = 0;

float var1;       // data received from IMU
float var2;
float IMUroll;
float IMUpitch;
int check;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;         // time constant for timer
  
double Pk1 = 3.5;    // 3.5       // Reaction wheel balancing PID values
double Ik1 = 30;     // 30
double Dk1 = 0.01;   // 0.01

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - motor position

double Pk2 = 1.1;           // Drive wheel balancing PID values
double Ik2 = 5.5;     
double Dk2 = 0.01;   

double Setpoint2, Input2, Output2;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup - motor position

void setup() {

  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  pinMode(A0, INPUT);
  pinMode(41, INPUT_PULLUP);
  pinMode(43, INPUT_PULLUP);
  pinMode(45, INPUT_PULLUP);
  pinMode(47, INPUT_PULLUP);

  pinMode(3, OUTPUT);       // PWM fan
  pinMode(4, OUTPUT);       // PWM fan

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-100, 100);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-100, 100);
  PID2.SetSampleTime(10);

  radio.begin();
  radio.openWritingPipe(addresses[0]); // 00002
  radio.openReadingPipe(1, addresses[1]); // 00001
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

}

void loop() {

    currentMillis = millis();
    if (currentMillis - previousMillis >= 10) {  // start timed event

      previousMillis = currentMillis;

      pot = analogRead(A0);
      pot2 = analogRead(A1);
      but1 = digitalRead(41);
      sw1 = digitalRead(45);
      sw2 = digitalRead(47);
      
      trimPot = (float) (pot - 512) / 100;    // read trim pot and scale to degrees
      trimPot2 = (float) (pot2 - 512) / 100;    // read trim pot and scale to degrees

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

      // check for radio data
      if (radio.available()) {
              radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));   
      } 

      else {
        Serial.println("no data");
      }

      // threshold remote data
      // some are reversed based on stick wiring in remote
      RFB = thresholdStick(mydata_remote.RFB);
      RLR = thresholdStick(mydata_remote.RLR);
      LT = thresholdStick(mydata_remote.LT);

      RFB = (RFB/100)*-1;    // scale to degrees   

      // *** Reaction wheel ***

      Input1 = IMUroll;                 // PID input is IMU data
      Setpoint1 = trimPot - accum;      // PID setpoint is trim pot minus the observation controller output

      accum = accum - (Output1/150);    // observation controller
      accum = (constrain(accum,-0.6,0.6));  // constrain the data    
      
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

      // *** Drive wheel ***

      Input2 = IMUpitch;                 // PID input is IMU data
      Setpoint2 = trimPot2 + RFB;        // PID setpoint is trim pot + remote data

      //accum2 = accum2 - (Output2/150);    // observation controller
      //accum2 = (constrain(accum2,-0.6,0.6));  // constrain the data 
      
      PID2.Compute();                   // compute PID

      velocity2 = Output2;        
    
      if (sw2 == 1) {                 // disable motor
        odrive1.SetVelocity(1, 0);
      }
    
      else {                          // drive motor
        odrive1.SetVelocity(1, velocity2);
      }

      // fans

      LT = map(LT,-460,460,-255,255);

      Serial.println(LT);
      
      if (LT > 0) {
         analogWrite(3, LT);
         analogWrite(4, 0);
      }

      else if (LT < 0) {
         LTa = abs(LT);
         analogWrite(4, LTa);
         analogWrite(3, 0);
      }

      else {
         analogWrite(3, 0);
         analogWrite(4, 0);
      }
         

    }   // end of timed loop

}
