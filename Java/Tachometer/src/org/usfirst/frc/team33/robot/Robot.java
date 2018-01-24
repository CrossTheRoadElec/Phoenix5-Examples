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
		_magTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		

		/* read section 7.9 Tachometer Measurement in software reference manual */
		// Talon Tach to read test values
		_tachTalon.configSelectedFeedbackSensor(FeedbackDevice.Tachometer, 0, 10);
		// 2 Edges per cycle (WHITE-black-WHITE-black)
		int edgesPerCycle = 2;
		_tachTalon.configSetParameter(430, edgesPerCycle, 0, 0, 10);
		// additional filtering if need be.
		int filterWindowSize = 1;
		_tachTalon.configSetParameter(431, filterWindowSize, 0, 0, 10);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		// Set talon to joystick value
		_magTalon.set(ControlMode.PercentOutput, _joy.getY());
		
		/* get the velocities of two talons,
		 * one uses quadrature (mag encoder), the other uses Talon-Tach */
		double magVel_UnitsPer100ms = _magTalon.getSelectedSensorVelocity(0);
		double tachVel_UnitsPer100ms = _tachTalon.getSelectedSensorVelocity(0);

		/* convert to RPM */
		// https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
		//MagRPM = magVel [units/kT] * 600 [kTs/minute] / 4096(units/rev), where kT = 100ms
		double magVelRPM = magVel_UnitsPer100ms * 600 / 4096;
		//TachRPM = tachVel [units/kT] * 600 [kTs/minute] / 1024(units/rev), where kT = 100ms
		double tachRPM = tachVel_UnitsPer100ms * 600 / 1024;

		// Print readings
		System.out.println("Mag encoder is: " + magVelRPM + "\t" + "Tachometer is: " + tachRPM);
	}
}
