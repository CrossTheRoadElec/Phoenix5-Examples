# CANifier Demo
The CANifier Demo project is an example that demonstrates some of the use cases of the CANifier. The CANifier is a CAN-controlled multipurpose LED and General Purpose Input/Output (GPIO) controller. It supports a variety of sensors that communicate through Quadrature, Limit Switch, SPI, I2c, and PWM Input/Output. With the Three low-side outputs, we are able to control any common RGB LED strip. The project also uses PWM inputs to communicate with a LIDAR Lite V3 sensor. Lastly, this project provides insight on how the new CTRE framework works and gives an example to how classes, schedulers, and organization works. 

# The Project's Hardware
This example project requires a [CANifier][2], [LIDAR Lite V3][3], [gamepad][4], RoboRIO, PWM motor controller, any common RGB LED Strip, and power source such as a power supply.

It is recommended that the user should test the CANifier in isolation prior to installation. It would also be the best opportunity for the user to field-upgrade the CANifier, test peripherals, and ensure wiring is correct. 

For transmitting signals, the CANifier is connected to both the LED strip and PWM motor controller. The LED Strip is wired to the CANifier's four pins on the labeled, A, B, C, and V+, which can be seen below in Figure 1.

#### Figure 1: LED strip wired to the CANifier, R to A, G to B, B to C, and + to V+.
![alt text][LED Strip]

The PWM motor controller’s PWM signal is wired directly to the CANifier. Wire the PWM motor controller’s ground to one of the ground pins available and connect the PWM wire (typically denoted by a white wire) to one of the 3 PWM outputs. As of Phoenix Toolsuite 5.0.3.3 Installer, PWM I/O pin 4 can only be used an input.

For retrieving signals, the project has a LIDAR Lite V3 Distance Sensor communicating with the CANifier through PWM. To wire the LIDAR Lite V3 to the CANifier, follow the steps below and use the Figure 2 as a guide. Note: The LIDAR Lite V3 is not required to use the example, but only enhances the user’s experience.

#### Figure 2: LIDAR Lite V3 wired to the CANifier. Steps with order explained in table.
![alt text][LIDAR]

The CANifier can then be connected to the RoboRIO through CAN. Once all connections to the CANifier has been made, it is recommended that it be insulted in a 1" heat shrink tubing as seen below in the Figure 3.

#### Figure 3: Heat shrink to insulate CANifier from shorts
![alt text][Insulation]

##### Further instructions on how to use and wire the CANifier can be found within the documentation, which can be found here. [http://www.ctr-electronics.com/can-can-canifier-driver-led-driver-gpio.html#product_tabs_technical_resources]

### Example Setup
Below is a sample setup containing all the hardware required. A PWM motor controller was not available for use at the time, so we modified the TalonSRX to become a PWM motor Controller. Below in Figure 4, we have the physical setup used to run the CANifier Demo project. 

#### Figure 4: Example Setup of all the hardware connected to each other
![alt text][Setup]

# The Project's Software
Before deploying the project to the RoboRIO, we need to make sure we have the correct firmware. The example was built using CTRE's Alpha build of the Phoenix Framework, which can be found on the [HERO's technical resources page][5]. Once the installer has been ran, it will be important to ensure all hardware has been flashed with the latest firmware. 

Once all hardware has been updated and libraries has been installed, User may deploy the example project to the RoboRIO. The project operates in three modes, which are explained below.
### Animate LED Strip
This is the default LED strip control mode, which will automatically start when the robot is enabled. All it does is ramp through the outer rim of the HSV color wheel at half brightness and is converted into RGB within the TaskHSV class. User can enter this mode by pressing button 6 (Right bumper) on the gamepad. 
### Direct Control LED Strip
In this mode, user can control the LED strip by using the gamepad joysticks. It takes the x and y values from the gamepad and finds both the angle and magnitude, which will represent the hue and saturation on the HSV color wheel, with a constant brightness. User can enter mode by pressing button 5 (Left bumper) on the gamepad.
### LIDAR Control LED Strip 
In this mode, user can control the LED strip by using the LIDAR Lite V3. The TaskMeasurePulseSensors retrieves the PWM signals from the LIDAR. Depending on how far the object is, the LED strip will illuminate at a different hue with fixed saturation and brightness. Green represents the closest with blue being the furthest. User can enter this mode by pressing button 7 (Left-trigger) on the gamepad.

#### Figure 5: LIDAR being used to provide hue based on distance detected
![alt text][More Pretty]

### The PWM Motor Controller
By default, the code has the PWM motor controller disabled for safety but can be enabled by user within the TaskMainLoop class file. Set the code below to true. The PWM motor controller only looks at the Y value from the gamepad and can go forward and backwards.
```c++
	/* No method of determining if controller is connected */
	if (controllerState == false){
        Schedulers::PeriodicTasks->Stop(Tasks::taskPWMmotorController);
	}
	else{
		    //Just for testing, can be enabled by user by changing controllerState to true */
        Schedulers::PeriodicTasks->Start(Tasks::taskPWMmotorController);
	}

```
#### Figure 6: PWM motor controller going forward is backwards as shown by the LED colors (It is a talon used as a PWM motor controller)
![alt text][Final Image]

The new Phoenix Framework has been designed to allow users to create tasks to enter them within a scheduler to create a more organized workspace. Take notice that all task classes were created under the tasks folder and created within the Tasks class under Platform class  as seen in the code below. TaskMainLoop was the task in charge of selecting which LED strip mode to operate in. 
```c++
/* Tasks */
TaskAnimateLEDStrip* Tasks::taskAnimateLEDStrip = new TaskAnimateLEDStrip();
TaskDirectControlLEDStrip* Tasks::taskDirectControlArm = new TaskDirectControlLEDStrip();
TaskPWMmotorController* Tasks::taskPWMmotorController = new TaskPWMmotorController();
TaskMeasurePulseSensors* Tasks::taskMeasurePulseSensors = new TaskMeasurePulseSensors();
TaskLIDARControlLEDStrip* Tasks::taskLIDARControlLEDStrip = new TaskLIDARControlLEDStrip();
TaskHSV* Tasks::taskHSVControlLedStrip = new TaskHSV();
TaskMainLoop* Tasks::taskMainLoop = new TaskMainLoop();

/* TaskList */
std::vector<CTRE::Tasking::ILoopable*> Tasks::FullList = {
		taskAnimateLEDStrip,
		taskDirectControlArm,
		taskPWMmotorController,
		taskMeasurePulseSensors,
		taskLIDARControlLEDStrip,
		taskHSVControlLedStrip,
		taskMainLoop,
};
```

Using the list created, the main Robot class then adds each task to the concurrent scheduler.
```c++
	void TeleopInit() {
		/* Add each task to the concurrent scheduler */
		for(auto ILoopable : Tasks::FullList){
			Schedulers::PeriodicTasks->Add(ILoopable);
		}
	}
```

Once all tasks were added to the concurrent scheduler, we then continuously process the concurrent scheduler that then processes each of the tasks.
```c++
	void TeleopPeriodic() {
		/* Run forever */

		/* Process the scheduler to process each task/loop */
		Schedulers::PeriodicTasks->Process();
	}
```

# Troubleshoot
### Why is the PWM motor controller not working?
As explained in documentation, the PWM motor controller is disabled by default for safety. It can be re-enabled by changing the state from false to true as shown in the code block above

### CANifier or Gamepad not recognized?
This may be due to having mismatched device ID's to the software. Users have two options. They may change the device ID to match the software or provide the correct device ID within the Platform class under the Platform folder. 
```c++
/* Hardware */
CTRE::CANifier* Hardware::canifier = new CTRE::CANifier(0);
frc::Joystick* Hardware::gamepad = new frc::Joystick(0);
}
``` 
 [1]: http://www.ctr-electronics.com/hro.html#product_tabs_technical_resources
 [2]: http://www.ctr-electronics.com/can-can-canifier-driver-led-driver-gpio.html
 [3]: https://www.sparkfun.com/products/14032
 [4]: http://www.andymark.com/product-p/am-2064.htm
 [5]: http://www.ctr-electronics.com/hro.html#product_tabs_technical_resources
 [LED Strip]: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/gh-doc/Java/CANifier%20Demo/Documentation/Capture2.PNG
 [LIDAR]: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/gh-doc/Java/CANifier%20Demo/Documentation/Capture.PNG
 [Insulation]: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/gh-doc/Java/CANifier%20Demo/Documentation/IMG_20171011_103615.jpg
 [Setup]: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/gh-doc/Java/CANifier%20Demo/Documentation/IMG_20171011_103545.jpg
 [More Pretty]: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/gh-doc/Java/CANifier%20Demo/Documentation/IMG_20171011_125250.jpg
 [Final Image]: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/gh-doc/Java/CANifier%20Demo/Documentation/IMG_20171011_130436.jpg
