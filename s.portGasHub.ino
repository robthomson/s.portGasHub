
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
// a 100pf capacitor can also be of benifit to filter the signal effecively.
// reality is that best solution is an optical filter; or not taking the signal off the CDI
// and rather using a second hall sensor pickup of feeding off the existing.
// CDI is just noisy and without good filtering will sometimes screw up the readings.

#include <SPort.h>                  //Include the SPort library
#include <FreqMeasure.h>
#include <arduino-timer.h>

#define SPORT_PIN 9         //a digital pin to send s.port data to frsky receiver.
#define VOLTAGE_PIN1 A6     //analog pin used to read a voltage from receiver battery
#define VOLTAGE_PIN2 A7     //analog pin used to read a voltage from the ignition battery
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


//timer
auto timer = timer_create_default();

//RPM
unsigned long rpmHZ = 0;
int lastRPMms = 0;

int lastRPM = 0;
int lastRPMHZ = 0;
int lastIG = 0;

// voltage divider ration - this is used as a multiplier to go from the 3v signal to the real received signal
float lastV1 = 0;
float lastV2 = 0;


// sensor values
float sensorValue1;
float sensorValue2;
float sensorValue3;
float sensorValue4;
float sensorValue5;
float dividerRatio = 6.065;

void setup() {

  //analogReference(INTERNAL);
  
  hub.registerSensor(sensor1);       //Add sensor to the hub
  hub.registerSensor(sensor2);       //Add sensor to the hub
  hub.registerSensor(sensor3);       //Add sensor to the hub
  hub.registerSensor(sensor4);       //Add sensor to the hub
  hub.registerSensor(sensor5);       //Add sensor to the hub
  

  hub.begin();                        //start the s.port transmition
  FreqMeasure.begin();                // start measuring rpm
  timer.every(5000, updateVoltages);  //update voltage every 5 seconds
  timer.every(1000, updateRPMs);       //update rpm value ever 1 second

}

double sum=0;
int count=0;


void loop() {


            // handle voltage reading
            // we need to constantly read this and return to a temp var that
            // we return to s.port every couple of seconds.
            // if we dont constantly read; we end up with bad values being sent.
            
            sensorValue1 =  analogRead(VOLTAGE_PIN1);
            float voltage1 = sensorValue1 * 0.0048875855327468;
            lastV1 = (voltage1 * dividerRatio) * 100;

            sensorValue2 =  analogRead(VOLTAGE_PIN2);
            float voltage2 = sensorValue2 * 0.0048875855327468;
            lastV2 = (voltage2 * dividerRatio) * 100;

            //grab the latest rpm value and process it.
            if (FreqMeasure.available()) {
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
           
            //RETURN CDI RPM VALUE to s.port
            sensorValue3 = (rpmHZ * 60);
            lastRPM = sensorValue3; 
            
            //RETURN CDI HZ VALUE to s.port
            sensorValue4 = rpmHZ;
            lastRPMHZ =  sensorValue4;     

            //RETURN ENGINE RUNNING/NOT RUNNING to sport
            if(rpmHZ <= 2){
                 sensorValue5 = 0;
            } else {
                sensorValue5 = 1;
            }
            lastIG = sensorValue5;     

            hub.handle();    
            timer.tick();
        
}

bool updateRPMs(){
    sensor3.value = lastRPM; 
    sensor4.value =  lastRPMHZ;   
    sensor5.value = lastIG;    
    return true;
}

bool updateVoltages(){
        sensor1.value = lastV1;           
        sensor2.value = lastV2;  
        return true;
}
