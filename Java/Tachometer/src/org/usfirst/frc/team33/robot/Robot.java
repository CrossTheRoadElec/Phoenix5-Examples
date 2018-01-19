/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team33.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	// Create a talon to measure the tach and the mag encoder
	TalonSRX _magTalon = new TalonSRX(4);
	TalonSRX _tachTalon = new TalonSRX(5);

	// Joystick to control motor
	Joystick _joy = new Joystick(0);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void teleopInit() {
		// Mag Encoder to read true values
		_magTalon.configSelectedFeedbackSensor(
				FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		
		
		// Talon Tach to read test values
		_tachTalon.configSelectedFeedbackSensor(
				FeedbackDevice.Tachometer, 0, 10);
		// 2 Edges per cycle (WHITE-black-WHITE-black)
		_tachTalon.configSetParameter(430, 2, 0, 0, 10);
		// 1 cycle per rotation
		_tachTalon.configSetParameter(431, 1, 0, 0, 10);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		// Set talon to joystick value
		_magTalon.set(ControlMode.PercentOutput, _joy.getY());

		// magRPM: u/100ms * 600(100ms/m) / 4096(rev/u) = rev/m
		double magVelRPM = _magTalon.getSelectedSensorVelocity(0) * 600 / 4096;
		// tachRPM: u/100ms * 600(100ms/m) / 1024(rev/u) = rev/m
		double tachRPM = _tachTalon.getSelectedSensorVelocity(0) * 600 / 1024;

		// Print readings
		System.out.println("Mag encoder is: " + magVelRPM + "\t"
				+ "Tachometer is: " + tachRPM);
	}
}
