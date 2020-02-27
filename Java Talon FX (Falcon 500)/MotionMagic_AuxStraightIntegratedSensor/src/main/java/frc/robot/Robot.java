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
 * The MotionMagic_AuxStraightIntegratedSensor example demonstrates the new Talon/Victor auxiliary and 
 * remote features used to peform complex closed loops. This example has the robot performing 
 * Motion Magic with an auxiliary closed loop on Integrated Sensor Difference to keep the robot straight.
 * 
 * This example uses:
 * - 2x Falcon Integrated Sensor, one of both sides of robot for Auxiliary Closed Loop on heading
 * A Talon/Victor calculates the heading by taking the difference between both sesnors.
 * - 2x Falcon Integrated Sensor, One on both sides of robot for Primary Closed Loop on Position
 * A Talon/Victor calculates the distance by taking the average between both sensors.
 * 
 * This example has two modes of operation, which can be switched between with Button 2.
 * 1.) Arcade Drive
 * 2.) Motion Magic with Integrated Sensor sum and Drive Straight with Integrated Sensor
 * 	difference
 * 
 * Controls:
 * Button 1: When pressed, zero all sensors. Set integrated sensors' positions to 0.
 * Button 2: When pressed, toggle between Arcade Drive and Motion Magic.
 * Button 5(Left shoulder): When pushed, will decrement the smoothing of the motion magic down to 0
 * Button 6(Right shoulder): When pushed, will increment the smoothing of the motion magic up to 8
 * 	When toggling into Motion Magic, the current heading is saved and used
 * 	as the the closed loop target. Can be changed by toggling out and in again.
 * Left Joystick Y-Axis: 
 * 	+ Arcade Drive: Drive robot forward and reverse
 * 	+ Motion Magic: Servo robot forward and reverse [-6, 6] rotations
 * Right Joystick X-Axis: 
 *  + Arcade Drive: Turn robot left and right
 *  + Motion Magic: Not used
 * 
 * Gains for Motion Magic and Auxiliary may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon FX: 20.2.3.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class Robot extends TimedRobot {
	/** Hardware */
	TalonFX _leftMaster = new TalonFX(2);
	TalonFX _rightMaster = new TalonFX(1);
	Joystick _gamepad = new Joystick(0);
	
	/** Invert Directions for Left and Right */
	TalonFXInvertType _leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	TalonFXInvertType _rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

	/** Config Objects for motor controllers */
	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
	
	/** A couple latched values to detect on-press events for buttons */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];
	
	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _lockedDistance = 0;
	double _targetAngle = 0;

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing;

	@Override
	public void robotInit() {
		/* Not used in this project */
	}
	
	@Override
	public void teleopInit(){
		/* Disable all motor controllers */
		_rightMaster.set(TalonFXControlMode.PercentOutput, 0);
		_leftMaster.set(TalonFXControlMode.PercentOutput, 0);
		
		/* Set Neutral Mode */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor as local Integrated Sensor */
		_leftConfig.primaryPID.selectedFeedbackSensor =	TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();	// Local Feedback Source

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		_rightConfig.remoteFilter0.remoteSensorDeviceID =_leftMaster.getDeviceID(); // Device ID of Source
		_rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; // Remote Feedback Source
		
		/* Now that the Left sensor can be used by the master Talon,
		 * set up the Left (Aux) and Right (Master) distance into a single
		 * Robot distance as the Master's Selected Sensor 0. */
		setRobotDistanceConfigs(_rightInvert, _rightConfig);
		
		/* Setup difference signal to be used for turn when performing Drive Straight with encoders */
		setRobotTurnConfigs(_rightInvert, _rightConfig);

		/* Configure neutral deadband */
		_rightConfig.neutralDeadband = Constants.kNeutralDeadband;
		_leftConfig.neutralDeadband = Constants.kNeutralDeadband;
		
		/* Motion Magic Configurations */
		_rightConfig.motionAcceleration = 2000;
		_rightConfig.motionCruiseVelocity = 2000;

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftConfig.peakOutputForward = +1.0;
		_leftConfig.peakOutputReverse = -1.0;
		_rightConfig.peakOutputForward = +1.0;
		_rightConfig.peakOutputReverse = -1.0;

		/* FPID Gains for distance servo */
		_rightConfig.slot0.kP = Constants.kGains_Distanc.kP;
		_rightConfig.slot0.kI = Constants.kGains_Distanc.kI;
		_rightConfig.slot0.kD = Constants.kGains_Distanc.kD;
		_rightConfig.slot0.kF = Constants.kGains_Distanc.kF;
		_rightConfig.slot0.integralZone = Constants.kGains_Distanc.kIzone;
		_rightConfig.slot0.closedLoopPeakOutput = Constants.kGains_Distanc.kPeakOutput;
		_rightConfig.slot0.allowableClosedloopError = 0;

		/* FPID Gains for turn servo */
		_rightConfig.slot1.kP = Constants.kGains_Turning.kP;
		_rightConfig.slot1.kI = Constants.kGains_Turning.kI;
		_rightConfig.slot1.kD = Constants.kGains_Turning.kD;
		_rightConfig.slot1.kF = Constants.kGains_Turning.kF;
		_rightConfig.slot1.integralZone = Constants.kGains_Turning.kIzone;
		_rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_Turning.kPeakOutput;
		_rightConfig.slot1.allowableClosedloopError = 0;

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
		_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
		zeroSensors();
	}
	
	@Override
	public void teleopPeriodic() {
		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		forward = Deadband(forward);
		turn = Deadband(turn);
	
		/* Button processing for state toggle and sensor zeroing */
		getButtons(btns, _gamepad);
		if(btns[2] && !_btns[2]){
			_state = !_state; 		// Toggle state
			_firstCall = true;		// State change, do first call operation
			_targetAngle = _rightMaster.getSelectedSensorPosition(1);
			_lockedDistance = _rightMaster.getSelectedSensorPosition(0);
		}else if (btns[1] && !_btns[1]) {
			zeroSensors();			// Zero Sensors
		}
		if(btns[5] && !_btns[5]) {
			_smoothing--; // Decrement smoothing
			if(_smoothing < 0) _smoothing = 0; // Cap smoothing
			_rightMaster.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing value is: " + _smoothing);
		}
		if(btns[6] && !_btns[6]) {
			_smoothing++; // Increment smoothing
			if(_smoothing > 8) _smoothing = 8; // Cap smoothing
			_rightMaster.configMotionSCurveStrength(_smoothing);
			
			System.out.println("Smoothing value is: " + _smoothing);
		}
		System.arraycopy(btns, 0, _btns, 0, Constants.kNumButtonsPlusOne);
				
		if(!_state){
			if (_firstCall)
				System.out.println("This is Acade Drive.\n");
			
			_leftMaster.set(TalonFXControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			_rightMaster.set(TalonFXControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		}else{
			if (_firstCall) {
				System.out.println("This is Motion Magic with the Auxiliary PID using the difference between two encoders.");
				System.out.println("Servo [-6,6] rotations while also maintaining a straight heading.\n");

				/* Determine which slot affects which PID */
				_rightMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
				_rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			}
			
			/* Calculate targets from gamepad inputs */
			double target_sensorUnits = forward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel  + _lockedDistance;
			double target_turn = _targetAngle;
			
			/* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
			_rightMaster.set(TalonFXControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn);
			_leftMaster.follow(_rightMaster, FollowerType.AuxOutput1);
		}
		_firstCall = false;
	}
	
	/** Zero integrated encoders on Talon */
	void zeroSensors() {
		_leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		System.out.println("[Integrated Sensors] All sensors are zeroed.\n");
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
	void setRobotTurnConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
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
		 * aux (other side's) distance into a single robot heading.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   heading magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.

				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.

				Will a sensor sum or difference give us a positive heading?

				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.

					Phase | Term 0   |   Term 1  | Result
				Sum:  -1 *((-)Master + (+)Aux   )| OK - magnitude will cancel each other out
				Diff: -1 *((-)Master - (+)Aux   )| NOT OK - magnitude increases with forward distance.
				Diff: -1 *((+)Aux    - (-)Master)| NOT OK - magnitude decreases with forward distance
			*/

			masterConfig.sum0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1

			/*
				PID Polarity

				With the sensor phasing taken care of, we now need to determine if the PID polarity is in the correct direction
				This is important because if the PID polarity is incorrect, we will run away while trying to correct

				Will inverting the polarity give us a positive counterclockwise heading?

				If we're moving counterclockwise(+), and the master is on the right side and inverted,
				it will have a negative velocity and the auxiliary will have a negative velocity
				 heading = right + left
				 heading = (-) + (-)
				 heading = (-)
				Let's assume a setpoint of 0 heading.
				This produces a positive error, in order to cancel the error, the right master needs to
				drive backwards. This means the PID polarity needs to be inverted to handle this
				
				Conversely, if we're moving counterclwise(+), and the master is on the left side and inverted,
				it will have a positive velocity and the auxiliary will have a positive velocity.
				 heading = right + left
				 heading = (+) + (+)
				 heading = (+)
				Let's assume a setpoint of 0 heading.
				This produces a negative error, in order to cancel the error, the left master needs to
				drive forwards. This means the PID polarity needs to be inverted to handle this
			*/
			masterConfig.auxPIDPolarity = true;
		} else {
			/* Master is not inverted, both sides are positive so we can diff them. */
			masterConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
			/* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
			masterConfig.auxPIDPolarity = true;
		}
		/**
		 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
		 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
		 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
		 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
		 *  and motion profile trajectory points can range +-2 rotations.
		 */
		masterConfig.auxiliaryPID.selectedFeedbackCoefficient = Constants.kEncoderUnitsPerRotation / Constants.kEncoderUnitsPerRotation;
	 }
}
