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
 * The MotionProfile_AuxMotionProfileArc example demonstrates the new Talon/Victor Auxiliary and 
 * remote featuresused to perform complex closed loops. This example has the robot performing 
 * Motion Profile with an auxiliary closed loop on pigeon yaw, which is called Motion Profile Arc.
 * 
 * This example uses:
 * - 2x Quad Encoders, One on both sides of robot for Primary Closed Loop on Position
 * A Talon/Victor calculates the distance by taking the sum of average between both sensors.
 * - Pigeon IMU used for Auxiliary Closed Loop on Pigeon Yaw. 
 * 
 * This example has two modes of operation, which can be switched between with Button 2.
 * 1.) Arcade Drive
 * 2.) Motion Profile with Quadrature Encoders and Motion Profile Arc with Pigeon
 * 
 * Controls:
 * Button 1: When pressed, zero sensors. Set quadrature encoders' positions + Pigeon yaw to 0.
 * Button 2: When pressed, toggle between Arcade Drive and Motion Profile Arc
 * Button 6: When pressed, select direction and final heading of Motion Profile and run 
 * 	Motion Profile Arc
 * Left Joystick Y-Axis: 
 * 	+ Arcade Drive: Drive robot in forward and reverse direction
 * 	+ Motion Profile Arc: Select Direction of Motion profile, forward or reverse, when
 * 	Button 6 is pressed and held. 
 * Right Joystick X-Axis: 
 *  + Arcade Drive: Turn robot in left and right direction
 *  + Motion Profile Arc: Select Final Heading of Motion profile [0, 360] degrees when 
 * 	Button 6 is pressed and held.
 * 
 * Gains for Motion Profile and Arc may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * 	- Talon SRX: 4.X
 * 	- Victor SPX: 4.X
 * 	- Pigeon IMU: 4.13 (required for ribbon-cable usecase)
 * 	- CANifier: 4.X
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;

public class Robot extends TimedRobot {
	/** Hardware */
	TalonSRX _leftMaster = new TalonSRX(2);
	TalonSRX _rightMaster = new TalonSRX(1);
	VictorSPX _tempMaster = new VictorSPX(2);
	PigeonIMU _pidgey = new PigeonIMU(3);

	Joystick _gamepad = new Joystick(0);
	
	/** Latched values to detect on-press events for buttons and POV */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];

	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	
	/** Motion profile example manager*/
	MotionProfileExample  _motProfExample = new MotionProfileExample(_tempMaster);
	
	/** Used to help control the Motion Profile Close Looping */
	enum ButtonEvent {
		ButtonOff, 
		ButtonOffToOn, 
		ButtonOn, 
		ButtonOnToOff;
	}

	@Override
	public void robotInit() {
		/* Not used in this project */
	}
	
	@Override
	public void teleopInit(){
		/* Disable all motors */
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftMaster.set(ControlMode.PercentOutput,  0);
		_tempMaster.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		_rightMaster.configFactoryDefault();
		_leftMaster.configFactoryDefault();
		_tempMaster.configFactoryDefault();
		_pidgey.configFactoryDefault();
		
		/* Set neutral modes */
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		_tempMaster.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor as local QuadEncoder */
		_leftMaster.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,				// Local Feedback Source
													Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
													Constants.kTimeoutMs);					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		_rightMaster.configRemoteFeedbackFilter(_leftMaster.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												Constants.REMOTE_0,							// Source number [0, 1]
												Constants.kTimeoutMs);						// Configuration Timeout
		
		// /* Configure the Pigeon IMU to the other Remote Slot on the Right Talon */
		// _rightMaster.configRemoteFeedbackFilter(_pidgey.getDeviceID(),
		// 										RemoteSensorSource.Pigeon_Yaw,
		// 										Constants.REMOTE_1,	
		// 										Constants.kTimeoutMs);
		
		/* Setup Sum signal to be used for Distance */
		_rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);	// Feedback Device of Remote Talon
		_rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													Constants.PID_PRIMARY,
													Constants.kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		_rightMaster.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														Constants.PID_PRIMARY,		// PID Slot of Source 
														Constants.kTimeoutMs);		// Configuration Timeout
		
		// /* Configure Remote Slot 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
		// _rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,
		// 											Constants.PID_TURN,
		// 											Constants.kTimeoutMs);
		
		// /* Scale the Feedback Sensor using a coefficient (Configured for 3600 units of resolution) */
		// _rightMaster.configSelectedFeedbackCoefficient(	1,
		// 												Constants.PID_TURN,
		// 												Constants.kTimeoutMs);

		_tempMaster.configRemoteFeedbackFilter(_rightMaster.getDeviceID(),
												RemoteSensorSource.TalonSRX_SelectedSensor,
												Constants.REMOTE_0);
		_tempMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,
												Constants.PID_PRIMARY,
												Constants.kTimeoutMs);

		_tempMaster.configRemoteFeedbackFilter(_pidgey.getDeviceID(),
												RemoteSensorSource.Pigeon_Yaw,
												Constants.REMOTE_1,	
												Constants.kTimeoutMs);
		_tempMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1,
												Constants.PID_TURN,
												Constants.kTimeoutMs);
		
		/* Configure output and sensor direction */
		_tempMaster.setInverted(false);
		_leftMaster.setSensorPhase(true);
		_rightMaster.setInverted(true);
		_rightMaster.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		_tempMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_tempMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_tempMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
		_tempMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

		/* Configure neutral deadband */
		_rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		_tempMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		
		/* Motion Magic Configurations */
		_tempMaster.configMotionAcceleration(2000, Constants.kTimeoutMs);
		_tempMaster.configMotionCruiseVelocity(2000, Constants.kTimeoutMs);

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_tempMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_tempMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

		/* FPID Gains for Motion Profile servo */
		_tempMaster.config_kP(Constants.kSlot_MotProf, Constants.kGains_MotProf.kP, Constants.kTimeoutMs);
		_tempMaster.config_kI(Constants.kSlot_MotProf, Constants.kGains_MotProf.kI, Constants.kTimeoutMs);
		_tempMaster.config_kD(Constants.kSlot_MotProf, Constants.kGains_MotProf.kD, Constants.kTimeoutMs);
		_tempMaster.config_kF(Constants.kSlot_MotProf, Constants.kGains_MotProf.kF, Constants.kTimeoutMs);
		_tempMaster.config_IntegralZone(Constants.kSlot_MotProf, Constants.kGains_MotProf.kIzone, Constants.kTimeoutMs);
		_tempMaster.configClosedLoopPeakOutput(Constants.kSlot_MotProf, Constants.kGains_MotProf.kPeakOutput, Constants.kTimeoutMs);

		/* FPID Gains for turn servo */
		_tempMaster.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		_tempMaster.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		_tempMaster.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		_tempMaster.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		_tempMaster.config_IntegralZone(Constants.kSlot_Turning, Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
		_tempMaster.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_tempMaster.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
		_tempMaster.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		_tempMaster.configAuxPIDPolarity(false, Constants.kTimeoutMs);

		/* Initialize */
		_firstCall = true;
		_state = false;
		zeroSensors();
	}
	
	@Override
	public void teleopPeriodic() {
		/* Temp for first calls or button events */
		ButtonEvent bExecuteAction = ButtonEvent.ButtonOff;

		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		forward = Deadband(forward);
		turn = Deadband(turn);	
		
		getButtons(btns, _gamepad);	
		if (btns[2] && !_btns[2]) {
			_state =  !_state;
			_firstCall = true;
		}else if (btns[1] && !_btns[1]) {
			/* if button1 is just pressed */
			zeroSensors();
		}

		/* Update current button state */
		if (btns[6] && !_btns[6])
			bExecuteAction = ButtonEvent.ButtonOffToOn;
		else if (!btns[6] && _btns[6]) 
			bExecuteAction = ButtonEvent.ButtonOnToOff;
		else if (btns[6]) 
			bExecuteAction = ButtonEvent.ButtonOn;
 		else 
			bExecuteAction = ButtonEvent.ButtonOff;

		/* Update previous button array for tracking */
		System.arraycopy(btns, 0, _btns, 0, Constants.kNumButtonsPlusOne);
		
		if(!_state){
			if (_firstCall)
				System.out.println("This is a basic arcade drive.\n");
			
			_tempMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		}else{
			/* Calculate targets from gamepad inputs */
			boolean bMoveForward = (forward >= 0) ? true : false;
			/* positive right stick => negative heading target (turn to right), limit to [-90, 90 deg of current heading] */
			double finalHeading_units = Constants.kPigeonUnitsPerRotation * turn * -1.0 * 0.25;

			if (_firstCall) {
				System.out.println("This is Motion Profile Auxiliary, also known as MotionProfileArc using the Pigeon for turn");
				System.out.println("Additonal options for running Motion Profile, to be selected when Button 6 is pressed and held:");
				System.out.println("\t1.) Select direction (forward or reverse) of Motion Profile with Left Joystick Y-Axis");
				System.out.println("\t2.) Select final heading [-90, 90] deg to current heading of robot after Motion Profile completion with Right Joystick X-Axis");
				neutralMotors("Target not set yet.\n");

				/* Slots are selected in the profile, not via selectProfileSlot() */

			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				/* Do Nothing in this state */
			} else if (bExecuteAction == ButtonEvent.ButtonOffToOn) {
				neutralMotors("Motion Profile Initialized, Continue holding Button 6\n");
				zeroSensors();
				_motProfExample.reset();
				_motProfExample.start(finalHeading_units, bMoveForward);
			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				/* Configured for Motion Profile on Quad Encoders' Sum and Auxiliary PID on Pigeon IMU's Yaw */
				_tempMaster.set(ControlMode.MotionProfileArc, _motProfExample.getSetValue().value);
				_rightMaster.follow(_tempMaster, FollowerType.AuxOutput1);
			}
			/* Call this periodically, and catch the output. Only apply it if user wants to run MP. */
			_motProfExample.control();

			//System.out.println("reqHead: " + finalHeading_units);
		}
		_firstCall = false;
	}
	
	/* Zeroes all sensors on Talons */
	void zeroSensors() {
		_leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_pidgey.setYaw(0);
		System.out.println("[Quad Encoders + Pigeon] All sensors are zeroed.\n");
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
	
	void neutralMotors(String reason) {
		_tempMaster.neutralOutput();
		_rightMaster.neutralOutput();

		/* if caller is reporting WHY motors are being neutralized, report it */
		if (reason != null && reason.length() > 0) {
			System.out.print("  Motors are neutral, ");
			System.out.println(reason);
		}
	}
	
	/** Gets all buttons from gamepad */
	void getButtons(boolean[] btns, Joystick gamepad) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			btns[i] = gamepad.getRawButton(i);
		}
	}
}
