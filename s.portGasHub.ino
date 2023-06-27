
// this code relies heavily on this up-stream s.port project
// https://github.com/RealTadango/FrSky/blob/master/examples/SimpleSensor/SimpleSensor.ino
// you will need to add his libraries to your arduoino environment for it to work.
//
// Measuring voltage will need voltage dividers - you cannot send more than 3v to the arduino
// A pins.   Suggested divider ration is 10k / 2k.  This will be sufficient for a 2s lipo
// to be used for the pack and ignition system.
// 
// Most CDI units will output 5v to the signal line.  This is too high for for some arduino
// boards.  A 10K resistor on the RPM pin line will drop the 5v down to 3v.

#include <SPort.h>                  //Include the SPort library

#define SPORT_PIN 9         //a digital pin to send s.port data to frsky receiver.
#define VOLTAGE_PIN1 A2     //analog pin used to read a voltage from receiver battery
#define VOLTAGE_PIN2 A3     //analog pin used to read a voltage from the ignition battery
#define RPM_PIN 2           //digital pin with or without interrupt for measuring rpm from cdi unit.

//#define USE_INTERRUPTS    // measure rpm using interupts.  comment out to use a simple pin read.
                            // depending on cpu speed and max rpm; interupts can swamp the cpu
                            // in this scenario simply comment out the USE_INTERRUPTS define
                            // and things should play better.


#define SENSOR_ID1 0x5900   //unique id number for voltage read on VOLTAGE_PIN1
#define SENSOR_ID2 0x5901   //unique id number for voltage read on VOLTAGE_PIN2
#define SENSOR_ID3 0x5902   //unique id number for RPM sensom read on RPM_PIN


// initialise s.port and create 3 sensors
SPortHub hub(0x12, SPORT_PIN);           
SimpleSPortSensor sensor1(SENSOR_ID1);   
SimpleSPortSensor sensor2(SENSOR_ID2);   
SimpleSPortSensor sensor3(SENSOR_ID3);  


// variables used for rpm sensor 
int rpmPulse = 0;
unsigned long lastRead = 0;
unsigned long interval = 500;
int last_rpm;

// variables used by kalman filter to smooth throttle readings.
double kalman_q= 0.05;   
double kalman_r= 150;   

// voltage divider ration - this is used as a multiplier to go from the 3v signal to the real received signal
float dividerRatio = 6.065;

// sensor values
float sensorValue1;
float sensorValue2;
float sensorValue3;


void setup() {
  hub.registerSensor(sensor1);       //Add sensor to the hub
  hub.registerSensor(sensor2);       //Add sensor to the hub
  hub.registerSensor(sensor3);       //Add sensor to the hub
  
  pinMode(RPM_PIN, INPUT);           // enable rpm reading on pin
  
  #ifdef USE_INTERRUPTS  
  attachInterrupt(digitalPinToInterrupt(RPM_PIN),rpmPinInterrupt,CHANGE);  //if using interupts - attach to the pin
  #endif

  hub.begin();                      //start the s.port transmition
  
  //Serial.begin(115200); // enable serial port if code debugging.

}

void loop() {
   
        unsigned long time = millis();  // keep track of time


        #ifdef USE_INTERRUPTS  
        noInterrupts();
        #endif
            
            //GET VOLTAGE1
            sensorValue1 =  kalman_update1(analogRead(VOLTAGE_PIN1));
            float voltage1 = sensorValue1 * (5.0 / 1023.0);
            sensor1.value = (voltage1 * dividerRatio) * 100 ;      
      
            //GET VOLTAGE2
            sensorValue2 =  kalman_update2(analogRead(VOLTAGE_PIN2));
            //sensorValue2 =  analogRead(VOLTAGE_PIN2);
            float voltage2 = sensorValue2 * (5.0 / 1023.0);
            sensor2.value = (voltage2 * dividerRatio) * 100 ;     
    
            // RPM
            #ifndef USE_INTERRUPTS      
                readRPM();
            #endif
                if (millis() - lastRead >= interval) {
                  lastRead  += interval;
                  sensorValue3 = rpmPulse * 30;
                  sensor3.value = sensorValue3; 
                  rpmPulse = 0;
                }
            #ifdef USE_INTERRUPTS      
            interrupts();
            #endif

      hub.handle(); //keep s.port data current.

}

// kalman filter to 'denoise' the measurements.
float kalman_update1(float measurement)
{
  //static int lcnt=0;
  static float x=sensorValue1; //value
  static float p=100; //estimation error covariance
  static float k=0; //kalman gain

  // update the prediction value
  p = p + kalman_q;

  // update based on measurement
  k = p / (p + kalman_r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;

  return x;
}

//kalman filter to 'denoise' the measurements.
float kalman_update2(float measurement)
{
  //static int lcnt=0;
  static float x=sensorValue2; //value
  static float p=100; //estimation error covariance
  static float k=0; //kalman gain

  // update the prediction value
  p = p + kalman_q;

  // update based on measurement
  k = p / (p + kalman_r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;

  return x;
}

#ifdef USE_INTERRUPTS  
void rpmPinInterrupt()
{
  rpmPulse++;
}
#endif

#ifndef USE_INTERRUPTS  
void readRPM(){
      int this_rpm = digitalRead(RPM_PIN);
      if( this_rpm == 0 && last_rpm == 1){ 
          rpmPulse++;
      }
      last_rpm = this_rpm;
}
#endif
