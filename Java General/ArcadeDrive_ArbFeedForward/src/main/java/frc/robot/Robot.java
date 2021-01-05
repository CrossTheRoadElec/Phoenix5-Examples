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
 * The ArcadeDrive_AuxFeedForward example demonstrates the ability to create an Arcade
 * Drive based robot by using the Talon's and Victor's Feed Forward feature. 
 * 
 * The Auxiliary Feed Forward feature allows the user to apply more or less output on 
 * the motor controllers current control mode's output, regardless of it being open or closed. 
 * 
 * Controls:
 * Left Joystick Y-Axis: Drive robot forward and reverse
 * Right Joystick X-Axis: Turn robot right and left
 * 
 * Supported Version:
 * 	- Talon SRX: 4.0
 * 	- Victor SPX: 4.0
 * 	- Pigeon IMU: 4.0
 * 	- CANifier: 4.0
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.sim.PhysicsSim;

public class Robot extends TimedRobot {
	/** Hardware, either Talon could be a Victor */
	TalonSRX _leftMaster = new WPI_TalonSRX(2);
	TalonSRX _rightMaster = new WPI_TalonSRX(1);
	Joystick _gamepad = new Joystick(0);

	@Override
	public void simulationInit() {
		PhysicsSim.getInstance().addTalonSRX(_leftMaster, 0.75, 4000);
		PhysicsSim.getInstance().addTalonSRX(_rightMaster, 0.75, 4000);
	}

	@Override
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}

	@Override
	public void robotInit() {
		/* Not used in this project */
	}
	
	@Override
	public void teleopInit(){
		/* Ensure motor output is neutral during init */
		_leftMaster.set(ControlMode.PercentOutput, 0);
		_rightMaster.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		_leftMaster.configFactoryDefault();
		_rightMaster.configFactoryDefault();
		
		/* Set Neutral mode */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		
		/* Configure output direction */
		_leftMaster.setInverted(false);
		_rightMaster.setInverted(true);
		
		System.out.println("This is Arcade Drive using Arbitrary Feed Forward.");
	}
	
	@Override
	public void teleopPeriodic() {		
		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();		
		forward = Deadband(forward);
		turn = Deadband(turn);

		/* Arcade Drive using PercentOutput along with Arbitrary Feed Forward supplied by turn */
		_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);

		System.out.printf("L: %.4f R: %.4f L vel: %d R vel: %d\n", _leftMaster.getMotorOutputPercent(), _rightMaster.getMotorOutputPercent(),
			              _leftMaster.getSelectedSensorVelocity(), _rightMaster.getSelectedSensorVelocity());
	}

	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}
}
