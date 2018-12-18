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
 * The Tachometer example deomonstrates Talon Tach's ability to act as a Tachometer.
 * This example is designed to be used with Quadrature Encoder wired to a Talon,
 * allowing us to compare the Tachometer readings to Talon's Selected Sensor Reading.
 *  
 * The Talon Tach has been configured to find 2 edges per rotation with zero filtering.
 * Note that the Talon Tach, like many tachometers, lack resolution at low RPM's, but should
 * be configurable in a future updates.
 * 
 * Controls:
 * Left Joystcik Y-Axis: Drive Talon Forward and Reverse
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot {
	/* Create a talon to measure the tach and the mag encoder */
	TalonSRX _magTalon = new TalonSRX(4);
	TalonSRX _tachTalon = new TalonSRX(5);

	/* Joystick to control motor */
	Joystick _joy = new Joystick(0);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void teleopInit() {
        /* Nonzero to block the config until success, zero to skip checking */
        final int kTimeoutMs = 30;
		
        /* Mag Encoder to read true values */
        _magTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                                0, 
                                                kTimeoutMs);
		
		/**
		 * Read section 7.9 Tachometer Measurement in software reference manual 
		 */

		/* Talon Tach to read test values */
		_tachTalon.configSelectedFeedbackSensor(FeedbackDevice.Tachometer, 0, kTimeoutMs);
		/* 2 Edges per cycle (WHITE-black-WHITE-black) */
		int edgesPerCycle = 2;
		_tachTalon.configSetParameter(430, edgesPerCycle, 0, 0, kTimeoutMs);
		/* Additional filtering if need be */
		int filterWindowSize = 1;
		_tachTalon.configSetParameter(431, filterWindowSize, 0, 0, kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		/* Set talon to joystick value */
		_magTalon.set(ControlMode.PercentOutput, _joy.getY());
		
		/**
		 * Get the velocities of two talons,
		 * 1.) Uses quadrature (mag encoder)
		 * 2.) Uses Talon Tach
		 */
		double magVel_UnitsPer100ms = _magTalon.getSelectedSensorVelocity(0);
		double tachVel_UnitsPer100ms = _tachTalon.getSelectedSensorVelocity(0);

		/**
		 * Convert to RPM
		 * https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
		 * MagRPM = magVel [units/kT] * 600 [kTs/minute] / 4096(units/rev), where kT = 100ms
		 */
		double magVelRPM = magVel_UnitsPer100ms * 600 / 4096;
		/* TachRPM = tachVel [units/kT] * 600 [kTs/minute] / 1024(units/rev), where kT = 100ms */
		double tachRPM = tachVel_UnitsPer100ms * 600 / 1024;

		/* Print readings */
		System.out.println("Mag encoder is: " + magVelRPM + "\t" + "Tachometer is: " + tachRPM);
	}
}
