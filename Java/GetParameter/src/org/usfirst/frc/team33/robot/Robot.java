/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team33.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	// Create a talon
	TalonSRX _talon = new TalonSRX(2);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Configure parameters to talon to see later
		_talon.config_kP(0, 0.2, 10);
		_talon.configForwardSoftLimitThreshold(200, 10);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopInit() {
		// On teleop init, print the configured setting
		double kP_value = _talon.configGetParameter(ParamEnum.eProfileParamSlot_P, 0, 0);
		double fwdSoftLimitThres = _talon.configGetParameter(ParamEnum.eForwardSoftLimitThreshold, 0, 0);
		System.out.println("P gain is: " + kP_value);
		System.out.println("Software limit forward is " + fwdSoftLimitThres);
	}
}
