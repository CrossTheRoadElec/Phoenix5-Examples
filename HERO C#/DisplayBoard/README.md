# What is DisplayBoard?
DisplayBoard is an example that demonstrates how we can use [CTRE's Driver Module](http://www.ctr-electronics.com/gadgeteer-driver-module.html) to control a RGB LED strip using three channels The Driver Module provides six low-side outputs, where each channel can be controlled using general-purpose outputs on any Y connector of [HERO](http://www.ctr-electronics.com/hro.html). With the included Talon SRX firmware, you may control the Driver Module through a [Talon SRX's](http://www.ctr-electronics.com/talon-srx.htm) data port as well.  We use the [HSV (Hue, Saturation, and Value) cylindrical-coordinate](https://en.wikipedia.org/wiki/HSL_and_HSV) to represent our points in the RGB color model. This exampe can be operated with either a [gamepad](https://www.amazon.com/Logitech-940-000117-Gamepad-F710/dp/B0041RR0TW/ref=sr_1_1?ie=UTF8&qid=1494953942&sr=8-1&keywords=Logitech+f709) or a [Pigeon IMU](http://www.ctr-electronics.com/gadgeteer-imu-module-pigeon.html).

# What is needed for DisplayBoard?
This example comes with a few options as you can go basic using only a HERO, Driver Module, LED strip, and a couple wires to get everything connected, or go advanced by throwing in a Pigeon IMU or a Talon SRX as well to get the full experience.

### Full Setup
For the complete setup, a HERO is connected to both a Talon SRX and a Pigeon IMU through CAN. We then used a ribbon cable to connect the Driver Module to the Talon SRX's data port. We used channels 4 to 6 on the Driver Module, one for each of the RGB values, to connect the LED strip along with 12V power from a [Power Distribution Breakout](http://www.ctr-electronics.com/power-distribution-breakout.html). If you look at the image below, you can see that channels 4 to 6 are connected to the LED Strip where P4 is red, P5 is green, and P6 is blue. The Driver Module also has ground supplied to it for its low side output. We used the power distribution breakout to supply power to the Talon SRX and the HERO as well. **Note:** To use the Talon SRX directly with the Driver Module, you must flash your Talon SRX with the firmware included in this repository under "Talon Firmware". If using a Talon SRX to control both a Driver Module and Motor at the same time, you must disable the limit switching.

###### LED Strip: P4 is red, P5 is green, and P6 is blue; all connected to LED Strip. 12V power from power distribution breakout supplied to LED strip and ground supplied to Driver Module.
![alt text](https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/Worlds_Demo_Examples/HERO%20C%23/DisplayBoard/README%20Images/DriverStrip.jpg)

###### Driver Module option 1: Use a ribbon cable to connect the Driver Module to a Talon SRX. Talon SRX connects to HERO with CAN.
![alt text](https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/Worlds_Demo_Examples/HERO%20C%23/DisplayBoard/README%20Images/TalonDriver.jpg)
###### Driver Module option 2: Use a ribbon cable to connect the Driver Module to port 3 of HERO labeled "PY".
![alt text](https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/Worlds_Demo_Examples/HERO%20C%23/DisplayBoard/README%20Images/HeroDriver.jpg)

###### CAN: If you are using a Pigeon IMU or Talon SRX within your build, wire them through CAN and set the I.D. of both devices to 0.
![alt text](https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/Worlds_Demo_Examples/HERO%20C%23/DisplayBoard/README%20Images/CAN.jpg)

### Example Setup
When assembled, your build should have three wires from the Driver Module along with 12V power connecting to the LED Strip (see *Full Setup*). Ground should be supplied to 'GND' on the Driver Module. The Driver Module is then connected to either a Talon SRX or a HERO (Port 3) with a ribbon cable. If using a Talon SRX, it should be connected to HERO through CAN. If using a Pigeon IMU, it should be connected through CAN as well. The code was written so that both the Talon SRX and Pigeon IMU has the I.D. of 0. This can be changed in LifeBoat, or you may modify the code lines where the Talon SRX and Pigeon IMU objects are created. 12V power can be supplied through a battery or power supply, which then can be distributed with a power distribution breakout to the HERO, Talon SRX, and LED strip
###### Example Setup:
![alt text](https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/Worlds_Demo_Examples/HERO%20C%23/DisplayBoard/README%20Images/Setup.jpg)

# How to Use DisplayBoard
Once you have assembled your Display Board and flashed it, you may see one of three things. If the LED strip is cycling through colors automatically, it means DisplayBoard is in Controller Mode and your example is ready to use. If the LED strip is holding one color (typically Purple/Magenta), it means DisplayBoard is in Pigeon IMU Mode and your example is ready to use. If the LED strip is flashing on and off, it means DisplayBoard is in Pigeon IMU Mode and your Pigeon has not finished calibration or a Pigeon could not be found.

If you do not have a gamepad connected to HERO, the example will default to Pigeon IMU Mode. To switch between the two modes, use the Logitech F710 Gamepad to select either 'D' for Controller Mode, or 'X' for Pigeon IMU Mode. 

### Controller Mode
Controller Mode allows you to use the gamepad's joysticks to control the hue and brightness of the LED strip. 
* The left joystick controls the hue, where the angle will match the surface of a HSV cylinder.
* The right joystick controls brightness, which can only be modified when the left joystick is in use.
* If the left joystick is unused, the LED strip will begin color cycling at the last used brightness.
* To adjust the brightness of the color cycling:
  1. Use the left joystick to exit color cycling.
  2. At the same time, use the right joystick to select the brightness.
  3. Once you have found your desired brightness, hold the right joystick and let go of the left joystick.
  4. Color cycling should now operate at the desired brightness.
  
You can change the color sequence by modifying the _colorSequence array and color codes within ColorSequencer.cs

### Pigeon Mode
Pigeon IMU Mode allows you to pick up and tilt the DisplayBoard to get different colors.

Pigeon IMU Mode uses the Pigeon to find Pitch and Roll. We find the sine of pitch and roll to get an angle by using inverse tangent of these two values. We then get a hue value based on the angle of the DisplayBoard, one of the main components of the HSV 3D model. The HSV is then converted into RGB and outputted to the LED strip. When playing with the tilt feature, you will find that the arrangement of the colors will match the surface of the HSV 3D model. For example, you may tilt in one direction to find the color red. If you were to tilt in the polar opposite direction, you would then find the color cyan, the color of formed from the two other primaries not red.

# Build of Materials
For a list of materials, check out the DisplayBoard BOM file in this repository

# Troubleshoot / FAQ
### Why does the example not work when connected to a Talon SRX?
The example requires your Talon to be flashed to the firmware included within this repository under "Talon Firmware". If that does not fix your problem, check to see if your Talon I.D. is set to 0. If not, you can change the I.D. in LifeBoat or redefine the Talon within code.

### Why does Controller Mode not work?
Ensure that your gamepad is connected and set to 'D'. You may have to wake up the example by pressing a button on your gamepad.

### Why does Pigeon IMU Mode not work?
If your DisplayBoard is not working with a Pigeon IMU, the LED strip will be stuck flashing on and off. This either means your Pigeon is still trying to calibrate (keep it still to finish calibration) or a Pigeon could not be found. Pigeon IMU is connected through CAN, meaning we need to use the device I.D. of 0 to have the example work. If your Pigeon IMU is not set to 0, you can change the I.D. in LifeBoat or redefine the Pigeon within code

### Why do we use P6 to P4 instead of the other port connections on the Driver Module?
We chose to use P6 to P4 because it made the wiring cleaner, but you can actually use P2 to P6 for providing a low side output (P1 has not been configured yet). The code was written to use P6 to P4, but you can redefine the ports in LEDStripController.cs. Ground is required for the Driver Module to output a low signal. You can also use the Driver Module to drive a Talon with the use of the +V port to protect against high voltage inductive spikes. 

### Why do we use Hue, Saturation, and Value instead of the standard RGB values?
We are using the RGB values as we use a RGB LED Strip but in code, we use [HSV Values](https://en.wikipedia.org/wiki/HSL_and_HSV). It made more sense as we could keep value and saturation constant while modifying the hue with a joystick or Pigeon IMU. Once our HSV value has been found, we convert it back into RGB for the Driver Module.

### Why does my Talon SRX not drive correctly when connected to a Driver Module?
At the moment, in order to use a Talon SRX to drive a motor while connected to a Driver Module you need to disable limit switching on the Talon SRX. Examples of doing this in C# and Java/C++ can be seen below.

*C#*
```c#
            _Talon.ConfigLimitMode(TalonSrx.LimitMode.kLimitMode_SrxDisableSwitchInputs);
```

*C++*
```c++
            _Talon->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
```

*Java*
```java
            _Talon->enableLimitSwitch(false, false);
```             
