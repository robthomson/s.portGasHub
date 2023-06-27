A simple but usefull build to send two voltages and rpm to an frsky receiver.

this code relies heavily on this up-stream s.port project
https://github.com/RealTadango/FrSky/blob/master/examples/SimpleSensor/SimpleSensor.ino
you will need to add his libraries to your arduoino environment for it to work.

Measuring voltage will need voltage dividers - you cannot send more than 3v to the arduino
A pins.   Suggested divider ration is 10k / 2k.  This will be sufficient for a 2s lipo
to be used for the pack and ignition system.
 
Most CDI units will output 5v to the signal line.  This is too high for for some arduinoboards.  
A 10K resistor on the RPM pin line will drop the 5v down to 3v.
