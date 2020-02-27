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
 * - 2x Falcon 500 Integrated Sensors, One on both sides of robot for Primary Closed Loop on Position
 * A Talon/Victor calculates the distance by taking the sum of average between both sensors.
 * - Pigeon IMU used for Auxiliary Closed Loop on Pigeon Yaw. 
 * 
 * This example has two modes of operation, which can be switched between with Button 2.
 * 1.) Arcade Drive
 * 2.) Motion Profile with Integrated Sensors and Motion Profile Arc with Pigeon
 * 
 * Controls:
 * Button 1: When pressed, zero sensors. Set Integrated Sensors' positions + Pigeon yaw to 0.
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
 * 	- Talon FX: 20.2.3.0
 *  - Pigeon IMU: 20.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.FollowerType;

public class Robot extends TimedRobot {
	/** Hardware */
	TalonFX _leftMaster = new TalonFX(2);
	TalonFX _rightMaster = new TalonFX(1);
	PigeonIMU _pidgey = new PigeonIMU(3);

	Joystick _gamepad = new Joystick(0);

	/** Invert Directions for Left and Right */
	TalonFXInvertType _leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	TalonFXInvertType _rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

	/** Config Objects for motor controllers */
	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
	
	/** Latched values to detect on-press events for buttons and POV */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];

	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	
	/** Motion profile example manager*/
	MotionProfileExample  _motProfExample = new MotionProfileExample(_rightMaster);
	
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
		_rightMaster.set(TalonFXControlMode.PercentOutput, 0);
		_leftMaster.set(TalonFXControlMode.PercentOutput,  0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		_pidgey.configFactoryDefault();
		
		/* Set neutral modes */
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */

		/** Distance Configs */

		/* Configure the left Talon's selected sensor as integrated sensor */
		_leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Feedback Source

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		_rightConfig.remoteFilter0.remoteSensorDeviceID = _leftMaster.getDeviceID(); //Device ID of Remote Source
		_rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
		
		/* Now that the Left sensor can be used by the master Talon,
		 * set up the Left (Aux) and Right (Master) distance into a single
		 * Robot distance as the Master's Selected Sensor 0. */
		setRobotDistanceConfigs(_rightInvert, _rightConfig);/* FPID for Distance */

		_rightConfig.slot3.kF = Constants.kGains_MotProf.kF;
		_rightConfig.slot3.kP = Constants.kGains_MotProf.kP;
		_rightConfig.slot3.kI = Constants.kGains_MotProf.kI;
		_rightConfig.slot3.kD = Constants.kGains_MotProf.kD;
		_rightConfig.slot3.integralZone = Constants.kGains_MotProf.kIzone;
		_rightConfig.slot3.closedLoopPeakOutput = Constants.kGains_MotProf.kPeakOutput;



		/** Heading Configs */
		_rightConfig.remoteFilter1.remoteSensorDeviceID = _pidgey.getDeviceID();    //Pigeon Device ID
		_rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw; //This is for a Pigeon over CAN
		_rightConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice(); //Set as the Aux Sensor
		_rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 3600.0 / Constants.kPigeonUnitsPerRotation; //Convert Yaw to tenths of a degree

		/* FPID for Heading */
		_rightConfig.slot1.kF = Constants.kGains_Turning.kF;
		_rightConfig.slot1.kP = Constants.kGains_Turning.kP;
		_rightConfig.slot1.kI = Constants.kGains_Turning.kI;
		_rightConfig.slot1.kD = Constants.kGains_Turning.kD;
		_rightConfig.slot1.integralZone = Constants.kGains_Turning.kIzone;
		_rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_Turning.kPeakOutput;

		/* Config the neutral deadband. */
		_leftConfig.neutralDeadband = Constants.kNeutralDeadband;
		_rightConfig.neutralDeadband = Constants.kNeutralDeadband;

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftConfig.peakOutputForward = +1.0;
		_leftConfig.peakOutputReverse = -1.0;
		_rightConfig.peakOutputForward = +1.0;
		_rightConfig.peakOutputReverse = -1.0;

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

		_leftMaster.configAllSettings(_leftConfig);
		_rightMaster.configAllSettings(_rightConfig);
		
		/* Configure output and sensor direction */
		_leftMaster.setInverted(_leftInvert);
		_rightMaster.setInverted(_rightInvert);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _leftMaster.setSensorPhase(true);
        // _rightMaster.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

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
			
			_rightMaster.set(TalonFXControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			_leftMaster.set(TalonFXControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
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
				/* Configured for Motion Profile on Integrated Sensors' Sum and Auxiliary PID on Pigeon IMU's Yaw */
				_rightMaster.set(TalonFXControlMode.MotionProfileArc, _motProfExample.getSetValue().value);
				_leftMaster.follow(_rightMaster, FollowerType.AuxOutput1);
			}
			/* Call this periodically, and catch the output. Only apply it if user wants to run MP. */
			_motProfExample.control();

			//System.out.println("reqHead: " + finalHeading_units);
		}
		_firstCall = false;
	}
	
	/* Zeroes all sensors on Talons */
	void zeroSensors() {
		_leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_pidgey.setYaw(0);
		System.out.println("[Integrated Sensors + Pigeon] All sensors are zeroed.\n");
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
		_leftMaster.neutralOutput();
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

	/** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
	 void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot distance.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   distance magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.

				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.

				Will a sensor sum or difference give us a positive total magnitude?

				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.

					Phase | Term 0   |   Term 1  | Result
				Sum:  -1 *((-)Master + (+)Aux   )| NOT OK, will cancel each other out
				Diff: -1 *((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
				Diff: -1 *((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
			*/

			masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Diff0 - Diff1
		} else {
			/* Master is not inverted, both sides are positive so we can sum them. */
			masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
		}

		/* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
		   the real-world value */
		masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
	 }
}
