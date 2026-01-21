  Unofficial SteeringPad900F firmware

This unofficial repo fixes a few issues with the 3.2 firmware found on https://www.printables.com/model/1223875-steering-pad-900-f/files

Using the forked [ArduinoJoystickWithFFBLibrary by https://github.com/bogdan-dumitrescu](https://github.com/bogdan-dumitrescu/ArduinoJoystickWithFFBLibrary)

  My thresholds:

const int thresholds[7] = 
{
  220, // button 3
  355, // button 5
  477, // button 6
  161, // buttons 3 & 5
  180, // buttons 3 & 6
  258, // buttons 5 & 6
  140  // buttons 3 & 5 & 6
}; 
