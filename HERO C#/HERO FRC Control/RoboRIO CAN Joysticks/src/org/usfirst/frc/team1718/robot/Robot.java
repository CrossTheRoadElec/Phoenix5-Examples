/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team1718.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.can.CANJNI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	
	/*This project sends Gamepad Data over the CAN bus
	 * and loads the CTRE Phoenix libraries so CTRE CAN
	 * motor controllers can enable.
	 */
	
	Joystick gamepad0;
	Joystick gamepad1;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//We have to create a CTRE device to force the CTRE libraries to load.
		//If we don't do this, Motor Controllers won't enable.
		PigeonIMU _dummy = new PigeonIMU(60);
		
		gamepad0 = new Joystick(0);
		gamepad1 = new Joystick(1);
	}
	
	/**
	 * This function is for periodic code in all robot modes.
	 */
	@Override
	public void robotPeriodic(){
		CANGamepad.send(gamepad0);
		CANGamepad.send(gamepad1);
	}
}
