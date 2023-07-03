
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
// a 100pf capacitor can also be of benifit linked from the cdi pin to gnd.

#include <SPort.h>                  //Include the SPort library

#define SPORT_PIN 9         //a digital pin to send s.port data to frsky receiver.
#define VOLTAGE_PIN1 A2     //analog pin used to read a voltage from receiver battery
#define VOLTAGE_PIN2 A3     //analog pin used to read a voltage from the ignition battery
#define RPM_PIN 5           //digital pin 5 must be used for CDI timing.  Do not change


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

//rpmbits
volatile unsigned long totalCounts;
volatile bool nextCount;
volatile unsigned long Timer1overflowCounts;
volatile unsigned long overflowCounts;
unsigned int counter, countPeriod;

// variables used by kalman filter to smooth throttle readings.
double kalman_q= 0.05;   
double kalman_r= 150;   

// voltage divider ration - this is used as a multiplier to go from the 3v signal to the real received signal
float dividerRatio = 6.065;

// sensor values
float sensorValue1;
float sensorValue2;
float sensorValue3;
float sensorValue4;
float sensorValue5;

void setup() {
  hub.registerSensor(sensor1);       //Add sensor to the hub
  hub.registerSensor(sensor2);       //Add sensor to the hub
  hub.registerSensor(sensor3);       //Add sensor to the hub
  hub.registerSensor(sensor4);       //Add sensor to the hub
  hub.registerSensor(sensor5);       //Add sensor to the hub
  
  pinMode(RPM_PIN, INPUT);           // enable rpm reading on pin


  hub.begin();                      //start the s.port transmition
  
  Serial.begin(115200); // enable serial port if code debugging.

}

void loop() {

          startCount(1000);
          while(!nextCount) {
                hub.handle(); //keep s.port data current.
          }
        
            //GET VOLTAGE1
            sensorValue1 =  kalman_update1(analogRead(VOLTAGE_PIN1));
            float voltage1 = sensorValue1 * (5.0 / 1023.0);
            sensor1.value = (voltage1 * dividerRatio) * 100 ;      
      
            //GET VOLTAGE2
            sensorValue2 =  kalman_update2(analogRead(VOLTAGE_PIN2));
            //sensorValue2 =  analogRead(VOLTAGE_PIN2);
            float voltage2 = sensorValue2 * (5.0 / 1023.0);
            sensor2.value = (voltage2 * dividerRatio) * 100 ;     
   
           
            //RPM
            sensorValue3 = (totalCounts * 60);
            sensor3.value = sensorValue3; 

            Serial.println(totalCounts);

            //HZ
            sensorValue4 = totalCounts;
            sensor4.value = totalCounts;     

            //HZ
            if(sensorValue3 <= 60){
                 sensorValue5 = 0;
            } else {
                sensorValue5 = 1;
            }
            sensor5.value = sensorValue5;     

 

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

void startCount(unsigned int period)
{
  nextCount = false;
  counter = 0;
  Timer1overflowCounts = 0;
  countPeriod = period;
  
  //Timer 1: overflow interrupt due to rising edge pulses on D5
  //Timer 2: compare match interrupt every 1ms
  noInterrupts();
  TCCR1A = 0; TCCR1B = 0; //Timer 1 reset
  TCCR2A = 0; TCCR2B = 0; //Timer 2 reset
  TIMSK1 |= 0b00000001;   //Timer 1 overflow interrupt enable
  TCCR2A |= 0b00000010;   //Timer 2 set to CTC mode
  OCR2A = 124;            //Timer 2 count upto 125
  TIMSK2 |= 0b00000010;   //Timer 2 compare match interrupt enable
  TCNT1 = 0; TCNT2 = 0;   //Timer 1 & 2 counters set to zero
  TCCR2B |= 0b00000101;   //Timer 2 prescaler set to 128
  TCCR1B |= 0b00000111;   //Timer 1 external clk source on pin D5
  interrupts();
}
//=================================================================
ISR(TIMER1_OVF_vect)
{
  Timer1overflowCounts++;
}
//=================================================================
ISR (TIMER2_COMPA_vect)
{
  overflowCounts = Timer1overflowCounts;
  counter++;
  if(counter < countPeriod) return;

  TCCR1A = 0; TCCR1B = 0;   //Timer 1 reset  
  TCCR2A = 0; TCCR2B = 0;   //Timer 2 reset    
  TIMSK1 = 0; TIMSK2 = 0;   //Timer 1 & 2 disable interrupts

  totalCounts = (overflowCounts * 65536) + TCNT1;
  nextCount = true;
}
