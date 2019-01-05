/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The CANifier Demo example demonstrates the currently supported features of
 * CANifier board. In This project the CANifier is used to:
 * 1.) Directly control a LED Strip 
 * 2.) Read PWM Inputs 
 * 3.) Ouput PWM Signals for external peripherials (PWM motor controller)
 * 
 * The project incorporates LIDAR and Gamepad for simple LED control.
 * 
 * As defined by the constants, PWM output is generated on Channel 2, Pin 15
 * CANifier has both a 5v, 3.3v supply pin to power up the LEDs and LIDAR
 * 
 * Controls:
 * Button 6: Run TaskAnimateLEDStrip, automate through HSV color wheel
 * Button 5: Run TaskDirectControlLEDStrip, control HSV value based on left joystick position/angle
 * Button 7: Run TaskLIDAR_ControlLEDStrip, control HSV value based on distance detected by LIDAR
 * Left Joystick: Used in both TaskDirectControlLED Strip, where position of joystick represents
 * position on HSV Wheel, and TaskPWMmotorController, to drive motor forward and reverse when 
 * enabled.
 * 
 * Set gamepadPresent in TaskMainLoop.java to true to enable PWM output to PWM Motor Controller
 * 
 * Supported Version:
 * 	- Talon SRX: 4.0
 * 	- Victor SPX: 4.0
 * 	- Pigeon IMU: 4.0
 * 	- CANifier: 4.0
 */

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.ILoopable;
import frc.robot.Platform.*;

public class Robot extends TimedRobot {
	@Override
	public void robotInit() {
        /* Factory Default all hardware to minimize unexpected behaviour */
        Hardware.canifier.configFactoryDefault();   // Not necessary for CANifier use
	}

	@Override
	public void teleopInit() {
		/* Add each task to the concurrent scheduler */
		for (ILoopable loop : Tasks.FullList) {
			Schedulers.PeriodicTasks.add(loop);
		}
	}

	@Override
	public void teleopPeriodic() {
		/** Run forever */

		/* Process the concurrent scheduler which will process our tasks */
		Schedulers.PeriodicTasks.process();
	}
}
