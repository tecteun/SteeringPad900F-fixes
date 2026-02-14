  Unofficial SteeringPad900F firmware

⚠️ note: for interrupt mode, an additional wire needs to be bodged

This unofficial repo fixes a few issues with the 3.2 firmware found on https://www.printables.com/model/1223875-steering-pad-900-f/files

- Filters some judder from the x axis sensor (applyDeadband())
- improved force feedback
- more stable
- more buttons
- updated UI
- compatibility with BeamNG, snowrunner, assetto corsa, misterfpga 
- and more.. its nuts 🥜



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
