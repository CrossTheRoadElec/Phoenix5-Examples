# What is PixyDrive?
PixyDrive is an example robot project that demonstrates the features of [Pixy Camera](https://www.andymark.com/Pixy-CMUcam5-Smart-Vision-Sensor-Camera-p/am-3477.htm) when paired with a [Display Module](http://www.ctr-electronics.com/gadgeteer-display-module.html), as well as other CTRE products such as the [Pigeon IMU](http://www.ctr-electronics.com/gadgeteer-imu-module-pigeon.html), [HEROs](http://www.ctr-electronics.com/hro.html), and [Talons](http://www.ctr-electronics.com/talon-srx.html). The Pixy Camera is a color-detection based visual sensor that can target basic color patterns.  This example demonstrates how to use that information to produce a robot that can follow a target pattern.  In this use case, the pattern is displayed on a 128X160 pixel display, mounted to the rear of a second robot that is manually driven by an operator.

# Assembling the Robots
This example was designed for two robots to be in use, which we will call PixyBot and DisplayBot.  PixyBot refers the robot with the Pixy Camera, and DisplayBot refers to the robot with the LCD Display.  When running the demo, PixyBot will "follow" DisplayBot while maintaining a reasonably constant distance between the two.  DisplayBot can then be manually driven by an operator.

Both robots are controlled by an individual HERO, which control four Talon SRXs each, allowing the robot to perform Mecanum drive. Each individual Talon connects to one of four gear motors, which individually connect to a Mecanum wheel. Each Talon also has a unique I.D. to communicate over CAN with the HERO, which may be set with LifeBoat.

* Left Front Talon is 1
* Left Rear Talon is 2
* Right Rear Talon is 3
* Right Front Talon is 4

If your Talons are arranged in a different manner, the I.D. can be changed or Talons can be redefined within the code.

```cs
        static TalonSrx LeftRear = new TalonSrx(2);
        static TalonSrx RightRear = new TalonSrx(3);
        static TalonSrx LeftFront = new TalonSrx(1);
        static TalonSrx RightFront = new TalonSrx(4);
```

### PixyBot
PixyBot will need a Pixy Camera connected to port 8 of the HERO and a Pigeon IMU connected to the left rear Talon, however with a simple source change, any of the Talons can be used, or alternatively the Pigeon can be wired directly to CAN Bus.  To connect the Pixy Camera to HERO, first insert [0.100" (2.54mm) headers](https://www.pololu.com/product/965) into the 6-pin ribbon cable that came with the Pixy Camera.  Then solder wires from the headers to the [gadgeteer breakout module](http://www.ctr-electronics.com/breakoutmodule.html), which then attach to the HERO with a [ribbon cable](http://www.ctr-electronics.com/talon-srx-data-cable-4-pack.html). More details on how to connect the 6-pin Pixy Camera wire to a gadgeteer breakout module can be found [here](http://cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_NET_Gadgeteer). When assembling PixyBot, it is best to place the Pixy Camera in the front and upright position of the robot.
###### Camera Position: Camera positioned in the front and upright
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/PixyBot1.jpg)
###### Camera Connection: 6-pin ribbon cable to gadgeteer breakout module
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/PixyBot3.jpg)
###### Pigeon IMU Connection: Pigeon IMU connected to left rear Talon
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/PixyBot4.jpg)

### DisplayBot
DisplayBot only needs a Display Module connected to port 8 of the HERO. The Display Module would be best positioned on the backside of the robot with the Display Module's port on the top portion.
###### Display Module Position: Display Module should be positioned on the back of DisplayBot
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/DisplayBot1.jpg)
###### Display Module Connection: Port should be on the upper side; else, the colors will be flipped
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/DisplayBot2.jpg)

### Example of the two robots completed, where the left robot is the DisplayBot and the right robot is the PixyBot.
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/Bots0.jpg)

# Flashing the Robots
Once the two robots have been assembled, use the Hero PixyDrive example included in this GitHub repo to deploy the application to each robot. The C# code for both the robots are within the same project but separated by configurations. The configuration can be selected through either the configuration manager window or the drop-down menu in the standard toolbar. For example, if flashing PixyBot, select the CTRE_PixyBot configuration in the drop-down menu and press the "Start" button. Once finished with PixyBot, you would then flash DisplayBot with the CTRE_DisplayBot configuration.
###### Configuration Selection: One of the two methods of selecting robot configuration
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/ConfigurationSelection.png)

# Using PixyMon to configure Pixy
Now that both robots have been assembled and flashed, you will need to tune in most of Pixy Camera's settings, which you will need to use the PixyMon GUI for from Charmed Labs' website http://charmedlabs.com/default/

### Steps to take before teaching Pixy
Once PixyMon and has been downloaded, open the GUI and plug the Pixy Camera into the computer via the Pixy's USB connector. You may be prompted to update the firmware, and if that is the case, follow the directions to flash your Pixy Camera. Once you have finished flashing the camera, you should be getting a visual stream of what the Pixy "sees". If the image is not focused, you may need to manually it.  Put the camera into focus by turning the lens of the camera in either a clockwise or a counterclockwise direction. Once focus has been achieved, tighten the setscrew to lock the focus position. More information the Pixy's manual focus can be found [here](http://cmucam.org/projects/cmucam5/wiki/The_image_in_PixyMon_is_out_of_focus).


Before moving on to teaching Pixy an object, which in this case will be DisplayBot, we want to set a few settings as seen in the next four steps.
###### First: Set the color code mode to color codes only on the 'Expert' tab for better object detection and less false objects
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/PixyConfig1.png)
###### Second: Set the number of Max blocks to 2 and Max blocks per signature to 1 on the 'Blocks' tab for detecting only one color-coded object at a time
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/PixyConfig2.png)
###### Third: Set the Data out port to Arduino ICSP SPI on the 'Interface' tab to allow communication between Pixy and HERO
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/PixyConfig3.png)
###### Fourth: Deselect the three checkboxes on the 'Camera' tab to allow persistent configuration and manual control
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/PixyConfig4.png)

### Teaching the object to Pixy
Start by bringing the Display Module into view of the Pixy Camera. Then play with the brightness setting on the 'Signature tuning' tab while monitoring the display until the Display Module's colors stand out more than everything else in the surrounding image (Enable Display Module with controls explained below after being flashed). To adjust the brightness, go into the 'Camera' tab and enable Auto Exposure Correction. You can now go back and use the slider to get your desired brightness. Be sure to go back and disable Auto Exposure Correction to ensure your settings are saved.
###### Brightness Setting: Colors of Display Module appear more vibrant than the background
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/Teach1.png)

The next step is to teach the two signatures of the color-coded object, one color at a time.  The first color is selected by using “Set CC signature 1” from the drop-down menu of the action tab. PixyMon will take a still image and wait for you to select the Magenta portion with your mouse. Once completed, you will repeat those steps using “Set CC signature 2” for selecting the Green half. You should then see a block surrounding the two colors with a text label showing the signatures index and angle of object.
###### Setting Signatures: You can teach up to seven signatures at a time, but this example only uses two for Magenta and Green
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/Teach2.png)
###### Object Detection: With the settings and newly taught signatures, Pixy now detects an object
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/Teach3.png)

Next, enter default program mode by clicking on the house icon in the toolbar and ensure that the object is still being detected. Test signature detection by moving the display back and forth as well as across the Pixy to ensure object has been well taught. You may also use the LED on the Pixy Camera, as it will illuminate white if an object is found with brightness increasing the stronger the object detection. Else, if object detection is not to satisfaction you may play with Signature 1 Range, Signature 2 Range, and Brightness on the 'Signature tuning' tab.
###### Signature Tuning: The sliders correspond to which signature you taught, which should be 1 and 2
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/Teach4.png)

### IMPORTANT: If trying to communicate to HERO with Pixy plugged into computer, be sure to switch into 'Default Program' Mode (House Icon)

# Setting up the Example
Now that both robots have been flashed and the Pixy has been configured for the dual colored Display Module, you may begin running the example. The test setup starts with setting PixyBot behind DisplayBot, both facing forward. For best results place between 1/2ft to 1ft apart. Enable the Display Module by pressing Button 3 (Button B) on DisplayBot. Now enter PixyDrive Mode by pressing Button 3 (Button B) on PixyBot. Now only using DisplayBot, you can drive around and watch PixyBot follow DisplayBot from its initial position relative to the Display Module when PixyDrive was initialized. You can disable the Display Module with Button 2 (Button A) on DisplayBot and exit PixyDrive Mode with Button 1 (Button X) on PixyBot. Gains can be adjusted in code for better PixyDrive experience.
###### Setup: PixyBot sits about 1 foot behind DisplayBot with the Display Module's backlight enabled before PixyDrive is initialized
![alt text](https://github.com/CrossTheRoadElec/HERO-PixyDrive/blob/master/README_Pictures/Bots1.jpg)

# Controls
### DisplayBot
* DisplayBot operates in Mecanum drive by default and works out of the box with a Logitech F710 controller. 
* The B button enables backlight in the display module. 
* The A button disables backlight in the display module. 
* The Right joystick controls the turn/twist of the robot. 
* The Left joystick controls the forward/backward/right/left drive of the robot. 
* On the Display Module, there should be a split screen where half of it is Magenta and the other half is Green when enabled. 

### PixyBot
* PixyBot operates in Mecanum drive by default and works out of the box with a Logitech F710 controller. 
* The X button puts the robot in manual drive mode. 
* The B button puts the robot in PixyDrive mode. 
* The A button cycles the LED color on the Pixy Camera. 
* The Left bumper decreases the range between Pixy Camera and the Object in PixyDrive mode. 
* The Right bumper increases the range between Pixy Camera and the object in PixyDrive mode. 
* The Right joystick controls the turn/twist of the robot. 
* The Left joystick controls the forward/backward/right/left drive of the robot. 

# Note
This example was designed to follow the one object you taught it. You can teach other objects by clearing all signatures and teaching new signatures with the strategies described above. For single colored objects, change color code mode to either Disabled or Mixed mode.

# Supported subsystems/sensors
This example demonstrates processing several sensors and subsystems including...
* Talon SRX via CAN Bus for voltage-compensated motor control.  This also provides voltage measurement of the battery for low-battery detection.
* Pigeon IMU via Gadgeteer cable.  Using the Gyro (deg per sec) about Z is a convenient method for calculating the 'D' portion of the turning PD close-loop.
* HERO Development Board.  This is the "brain" of the robot where your C# code resides and executes.
* Pixy Camera.  This is the visual sensor, supported via an open source HERO C# driver.
* LCD Display.  Provides the target for PixyBot to follow.
* F710 Logitech Gamepad.  Provides convenient wireless control via a USB Dongle inserted into the HERO.  This also has the advantage of instant connection immediately after a power boot (with no setup effort).
##### ... However, this platform has been used for a variety of projects that leveraged...
* Preprogrammed maneuvers leveraging the IMU signals from the Pigeon (Yaw, Pitch, Roll, Accelerometer/Gyro, Hit Detection, etc.).
* Position closed loop using positions sensors (Talon Feedback connector).
* LED Strip control via the CTRE Driver Module.
* Distance closed loops leveraging sonar modules.
* ...and much more as the HERO has general support for I2C/SPI/UART/CAN Bus/etc.

# Build of Materials
For a list of materials and where you can find them, check out the Robot BOM file in this repository.

### CAD Files
Here you can find the CAD files for both the Display Module mount and the Pixy Camera mount in this repository under "CAD Files".


# Troubleshoot / FAQ
#### Why are the Robots not responding?
If you are using the Logitech F710 Controller, ensure that your controller is set on 'D' at the top. You also may need to wake up your robot by pressing any button.

#### Why does PixyDrive not turn or move very slowly?
If your robot responds very weakly when operating in PixyDrive mode, you may have to adjust the KD Gains or Correction Ratios found within PixyBot.cs. Quick tip for setting these gains is to double P until you get oscillation, then use d to dampen it.

#### Why is the Brightness Slider have no effect in PixyMon?
If you followed the 4 configuration steps when first running PixyMon, then Auto Exposure Correction should be disabled. You want to go back enable Auto Exposure Correction, which will allow you to adjust the brightness. Once you have found your setting, be sure to go back and disable Auto Exposure Correction to save your settings.

#### Why is the Robot driving weird?
Immediately, you should check to see if your Mecanum wheels are correctly positioned. If done correctly the wheels should be creating an X with the treads or all point towards the center. You Talons may also be assembled differently than what the code defaults to. Check to see if your I.D.s match:
* Left Front Talon is 1
* Left Rear Talon is 2
* Right Rear Talon is 3
* Right Front Talon is 4
If this is not the case, you can change the I.D.s with LifeBoat, rearrange them, or change the definitions in within PixyBot.cs and DisplayBot.cs

#### Why does the Robot jitter around when in PixyDrive Mode?
This is caused by false object detection which can happen when you have many objects in Pixy Camera's view. This can usually be resolved by changing the number of Maximum Blocks and Maximum Blocks per signature to 1 or 2 to only focus on one object. Another thing to note is that Pixy Camera will always grab the largest target, so if your background consists of Green and Magenta, you may have to change the colors of the Display Module or select a different object to follow.

#### Why does my Robot no longer respond to the object?
There are a few things that could cause this. 
* Lighting has a huge impact in object detection so if lighting is changed drastically, then you may have to adjust the brightness or even reteach the colors of your object. 
* You may have the of auto settings enabled, thus your previous setup was not saved. You can find these settings under the 'Camera' tab of PixyMon. 
* If you just plugged your Pixy into the computer and it no longer responds, it is because defaulted to Cooked Mode where you will see the background but there is no communication to the HERO. You will want to switch it back to Default Program Mode by clicking on the House Icon.
* You should also check to see if Data out port is set to Arduino ICSP SPI.  In testing the Pixy Camera did not work with the traditional SPI with CS/SS setting.
