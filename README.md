  Unofficial SteeringPad900F firmware

I have not made anything in this repo, currently this is just a clone of the firmware found at https://www.printables.com/model/1223875-steering-pad-900-f/files, all the credits go to Rui Caldas. 

This unofficial repo tries to fix a few issues with the 3.2 firmware

Using the forked [ArduinoJoystickWithFFBLibrary by https://github.com/bogdan-dumitrescu](https://github.com/bogdan-dumitrescu/ArduinoJoystickWithFFBLibrary)
(note: this fork seems to be bugged, for example springeffect is broken, I'm currently looking for a way to test/debug all ffb effect types)

  My thresholds:

```
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
```

Also, good to know that the reset button is beneath the oled display, on the right side, so just push it lightly to reset the pro micro.


