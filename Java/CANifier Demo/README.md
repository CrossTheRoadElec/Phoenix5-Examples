# CANifier Demo
The CANifier Demo project is an example that demonstrates some of the use cases of the CANifier. The CANifier is a CAN-controlled multipurpose LED and General Purpose Input/Output (GPIO) controller. It supports a variety of sensors that communicate through Quadrature, Limit Switch, SPI, I2c, and PWM Input/Output. With the Three low-side outputs, we are able to control any common RGB LED strip. The project also uses PWM inputs to communicate with a LIDAR Lite V3 sensor. Lastly, this project provides insight on how the new CTRE framework works and gives an example to how classes, schedulers, and organization works. 

# The Project's Hardware
This example project requires the CANifier (ADDLINKHERE), RoboRIO, LIDAR Lite V3 (ADDLINKHERE), a PWM motor controller, a gamepad (ADDLINKHERE), any common RGB LED Strip, and a power source such as a power supply.

It is recommended that the user should test the CANifier in isolation prior to installing it for usage. It will also be the best opportunity for the user to field-upgrade the CANifier, test peripherals, and ensure wiring is correct. 

The CANifier is wired directly to the RoboRIO though CAN. The LED Strip is wired to the CANifier's four pins on the edge labeled, A, B, C, and V+, which can be seen below in Figure 1. The PWM motor controller’s PWM signal can be connected directly to the CANifier. Wire the PWM motor controller’s ground to one of the ground pins available and connect the PWM wire (usually denoted by a white wire) to one of the 3 PWM Outputs. As of current firmware, PWM I/O pin 4 can only be used an input. The LED strip and PWM motor controller will serve as an example on how the CANifier can be used to output signals.

As for retrieving input signals, the project has a LIDAR Lite V3 Distance Sensor communicating with the CANifier through PWM. To wire the LIDAR Lite V3 to the CANifier, follow the steps below and use the Figure 2 as a guide.
1. Wire the LIDAR's 5V DC power (+) connection to the CANifier's 5V pin. (Red Wire)
2. Wire the LIDAR's ground to the CANifier's ground pin. (Black Wire)
3. Wire the LIDAR's mode-control connection to the CANifier's PWM input pin. (Yellow Wire)
4. Connect one side of a 1k resistor to the mode-control connection of the LIDAR.
5. Connect the other side of the 1k resistor to the trigger pin (ground).

Note: The LIDAR Lite V3 is not required to use the example, but only enhances the user’s experience.

##### Further instructions on how to use and wire the CANifier can be found within the documentation, which can be found here. (ADDLINKHERE)

### Example Setup
Below is a sample setup containing all the hardware required. A PWM motor controller was not available for use at the time, so we modified the TalonSRX to become a PWM motor Controller.

##Image here

# The Project's Software
Before looking at the project's files, we need to make sure we have the correct firmware. The example was built using CTRE's Alpha build of the Phoenix Framework, which can be found on the HERO's page (ADDLINKHERE). Once the installer has been ran, it will be important to ensure that the CANifier has the correct firmware to utilize all of its features. Firmware can be flashed through a HERO or Web dash. 

Once all the Hardware has been brought up to date, user may run the CANifier Demo project. 

Controls... Blah

