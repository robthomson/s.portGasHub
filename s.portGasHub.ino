
// this code relies heavily on this up-stream s.port project
// https://github.com/RealTadango/FrSky/blob/master/examples/SimpleSensor/SimpleSensor.ino
// you will need to add his libraries to your arduoino environment for it to work.
//
// FreqMeasure can be found here: https://www.pjrc.com/teensy/td_libs_FreqMeasure.html
//
// Timer https://github.com/contrem/arduino-timer
//
// Measuring voltage will need voltage dividers - you cannot send more than 3v to the arduino
// A pins.   Suggested divider ration is 10k / 2k.  This will be sufficient for a 2s lipo
// to be used for the pack and ignition system.
// 
// Most CDI units will output 5v to the signal line.  This is too high for for some arduino
// boards.  A 10K resistor on the RPM pin line will drop the 5v down to 3v.
// a 100pf capacitor can also be of benifit linked from the cdi pin to gnd.

#include <SPort.h>                  //Include the SPort library
#include <FreqMeasure.h>
#include <arduino-timer.h>

#define SPORT_PIN 9         //a digital pin to send s.port data to frsky receiver.
#define VOLTAGE_PIN1 A2     //analog pin used to read a voltage from receiver battery
#define VOLTAGE_PIN2 A3     //analog pin used to read a voltage from the ignition battery
#define RPM_PIN 8           //digital pin 8 must be used for CDI timing.  Do not change


#define SENSOR_ID1 0x5900   //unique id number for voltage read on VOLTAGE_PIN1
#define SENSOR_ID2 0x5901   //unique id number for voltage read on VOLTAGE_PIN2
#define SENSOR_ID3 0x5902   //unique id number for RPM sensom read on RPM_PIN (RPM)
#define SENSOR_ID4 0x5903   //unique id number for RPM sensor read on RPM_PIN (HZ)
#define SENSOR_ID5 0x5904   //unique id number for RPM sensor read on RPM_PIN (1 = running 0 = off) just a simple flag you can grab to notify of dead stick

// initialise s.port and create 3 sensors
SPortHub hub(0x12, SPORT_PIN);           
SimpleSPortSensor sensor1(SENSOR_ID1);   
SimpleSPortSensor sensor2(SENSOR_ID2);   
SimpleSPortSensor sensor3(SENSOR_ID3);  
SimpleSPortSensor sensor4(SENSOR_ID4);  
SimpleSPortSensor sensor5(SENSOR_ID5);  

// variables used by kalman filter to smooth throttle readings.
double kalman_q= 0.05;   
double kalman_r= 150;   

//RPM
unsigned long rpmHZ = 0;
int lastRPMms = 0;

// voltage divider ration - this is used as a multiplier to go from the 3v signal to the real received signal
float dividerRatio = 6.065;

// sensor values
float sensorValue1;
float sensorValue2;
float sensorValue3;
float sensorValue4;
float sensorValue5;

//timer
auto timer = timer_create_default();
  

void setup() {
  hub.registerSensor(sensor1);       //Add sensor to the hub
  hub.registerSensor(sensor2);       //Add sensor to the hub
  hub.registerSensor(sensor3);       //Add sensor to the hub
  hub.registerSensor(sensor4);       //Add sensor to the hub
  hub.registerSensor(sensor5);       //Add sensor to the hub
  
  //pinMode(12,OUTPUT);

  hub.begin();                      //start the s.port transmition

  FreqMeasure.begin();

  timer.every(5000, getVoltages);  //update voltage every 5 seconds

  Serial.begin(115200); // enable serial port if code debugging.

}

double sum=0;
int count=0;
void loop() {


 
            //SET VOLTAGE1
            sensor1.value = sensorValue1;   
      
            //SET VOLTAGE2
            sensor2.value = sensorValue2; 


            if (FreqMeasure.available() >= 2) {
                // average several reading together
                sum = sum + FreqMeasure.read();
                count = count + 1;
                if (count > 30) {
                  rpmHZ = FreqMeasure.countToFrequency(sum / count);
                  sum = 0;
                  count = 0;
                  lastRPMms = millis();
                }
              } 


            //time out the measurements if nothing for some time.
            if (FreqMeasure.available() <= 1) {
            if((millis() - lastRPMms) > 5000){
                rpmHZ = 0;         
            }
            }
           
            //CDI RPM VALUE
            sensorValue3 = (rpmHZ * 60);
            sensor3.value = sensorValue3; 
            
            //CDI HZ VALUE
            sensorValue4 = rpmHZ;
            sensor4.value =  sensorValue4;     

            //ENGINE RUNNING/NOT RUNNING
            if(rpmHZ <= 2){
                 sensorValue5 = 0;
            } else {
                sensorValue5 = 1;
            }
            sensor5.value = sensorValue5;     


            hub.handle();    
            timer.tick();
}

bool getVoltages(){
            sensorValue1= ((analogRead(VOLTAGE_PIN1) * (5.0 / 1023.0)) *  dividerRatio) * 100;
            sensorValue2 = ((analogRead(VOLTAGE_PIN2) * (5.0 / 1023.0)) * dividerRatio) * 100;            
            return true; 
}
