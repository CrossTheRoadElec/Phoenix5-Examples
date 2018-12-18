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
 * The CANifier Quadrature example demonstates CANifier's ability to read a Quadrature
 * Envocder's input and compares it to Talon SRX's Quadrature reading. 
 * CANifier pins used in example are:
 * 	5V:		pin 3
 * 	Quad A: pin 4
 * 	GND: 	pin 5
 * 	Quad B:	pin 6
 * 
 * Controls:
 * Left Joystick: Throttle Talon in forward and reverse direction
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	/* Hardware */
	CANifier _can = new CANifier(4);	// CANifier
	TalonSRX _tal = new TalonSRX(1);	// Talon SRX
	Joystick _joy = new Joystick(0);	// Joystick or Gamepad

	/* Variable to keep track of loop count */
	int count = 0;

	@Override
	public void robotInit() {
		/* Not used in this project */
	}

	@Override
	public void teleopInit() {
		/* Factory Default all hardware to prevent unexpected behaviour */
		_can.configFactoryDefault();
		_tal.configFactoryDefault();
        
	    /* nonzero to block the config until success, zero to skip checking */
    	final int kTimeoutMs = 30;

		/* Configure talon with feedback device to double check CANifier */
		_tal.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		
		/* On Teleop Initialization, set positions to some value */
		_tal.setSelectedSensorPosition(-745, 0, kTimeoutMs);
		_can.setQuadraturePosition(-745, kTimeoutMs);
		
		/* Configure velocity measurements to be what we want */
		_can.configVelocityMeasurementPeriod(VelocityPeriod.Period_100Ms, kTimeoutMs);
		_can.configVelocityMeasurementWindow(64, kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		/* Every 20th loop print */
		if(count++ >= 20) {
			/* CANifier */
			System.out.print(	"CANifier:\tPos: " + _can.getQuadraturePosition() +
								"\tVel: " + _can.getQuadratureVelocity());
		
			/* Formatting */
			System.out.print("\t||\t");
			
			/* TalonSRX */
			System.out.println(	"Talon:\tPos: " + _tal.getSelectedSensorPosition(0) + 
								"\tVel: " + _tal.getSelectedSensorVelocity(0));

			/* Extra line to deliniate each loop */
			System.out.println();
			
			/* Reset counter */
			count = 0;
		}
		
		/* Drive talon with joystick */
		_tal.set(ControlMode.PercentOutput, _joy.getY());
	}
}
