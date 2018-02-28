/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.*;
import com.ctre.phoenix.CANifier.VelocityPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	
	/* CANifier */
	CANifier _can;
	
	/* TalonSRX */
	TalonSRX _tal;
	
	/* Joystick */
	Joystick _joy;

	/* Variable to keep track of loop count */
	int count = 0;

	@Override
	public void robotInit() {
		/* Instantiate objects */
		_can = new CANifier(0);
		_tal = new TalonSRX(4);
		_joy = new Joystick(0);
	}

	@Override
	public void teleopInit() {
		/* Configure talon with feedback device to double check CANifier */
		_tal.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		
		/* On Teleop Initialization, set positions to some value */
		_tal.setSelectedSensorPosition(-745, 0, 0);
		_can.setQuadraturePosition(-745, 10);
		
		/* Configure velocity measurements to be what we want */
		_can.configVelocityMeasurementPeriod(VelocityPeriod.Period_100Ms, 0);
		_can.configVelocityMeasurementWindow(64, 0);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		/* Every 20th loop print */
		if(count++ >= 20) {
			/* CANifier */
			System.out.println("CANifier:\tPosition: " + _can.getQuadraturePosition() + "\tVelocity: " + _can.getQuadratureVelocity());
			
			/* TalonSRX */
			System.out.println("TAlon:\tPosition: " + _tal.getSelectedSensorPosition(0) + "\tVelocity: " + _tal.getSelectedSensorVelocity(0));

			/* Extra line to deliniate each loop */
			System.out.println();
			
			/* Reset counter */
			count = 0;
		}
		
		/* Drive talon with joystick */
		_tal.set(ControlMode.PercentOutput, _joy.getY());
	}
}
