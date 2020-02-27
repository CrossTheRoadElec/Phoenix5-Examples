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
 * The DriveStraight_AuxPigeon example demonstrates the new Talon/Victor auxiliary and 
 * remote features used to perform complex closed loops. This example has the robot
 * driving in Percent Output with an auxiliary closed loop on Pigeon Yaw to keep the robot straight.
 * 
 * This example uses:
 * - Talon FX's for drivetrain motors
 * - Pigeon IMU wired on CAN Bus for Auxiliary Closed Loop on Yaw
 * 
 * This example has two modes of operation, which can be switched between with Button 2.
 * 1.) Arcade Drive
 * 2.) Percent Output Drive Straight with Pigeon
 * 
 * Controls:
 * Button 1: When pressed, zero heading. Set Pigeon heading to 0.
 * Button 2: When pressed, toggle between Arcade Drive and Drive Straight with Pigeon.
 * 	When toggling into Drive Straight, the current heading is saved and used as the 
 * 	closed loop target. Can be changed by toggling out and in again.
 * Left Joystick Y-Axis: Drive robot forward and reverse in both control modes.
 * Right Joystick X-Axis: 
 * 	+ Arcade Drive: Turn robot left and right
 * 	+ Drive Straight: Not used
 * 
 * Gains for auxiliary closed loop may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * 	- Talon FX: 20.2.3.0
 *  - Pigeon IMU: 20.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Robot extends TimedRobot {
	/** Hardware */
	TalonFX _leftMaster = new TalonFX(2);
	TalonFX _rightMaster = new TalonFX(1);
	PigeonIMU _pidgey = new PigeonIMU(3);
	Joystick _gamepad = new Joystick(0);

	TalonFXInvertType _leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	TalonFXInvertType _rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
	
	/** Latched values to detect on-press events for buttons */
	boolean[] _previousBtns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] _currentBtns = new boolean[Constants.kNumButtonsPlusOne];
	
	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _targetAngle = 0;
	int _printCount = 0;

	@Override
	public void robotInit() {
		/* Not used in this example */
	}
	
	@Override
	public void teleopInit(){
		/* Disable all motor controllers */
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftMaster.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		_pidgey.configFactoryDefault();
		
		/* Set Neutral Mode */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);

		_leftMaster.setInverted(_leftInvert);
		_rightMaster.setInverted(_rightInvert);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _rightMaster.setSensorPhase(true);
        // _leftMaster.setSensorPhase(true);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the Pigeon IMU as a Remote Sensor for the right Talon */
		_rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw;
		_rightConfig.remoteFilter0.remoteSensorDeviceID = _pidgey.getDeviceID();
		
		/* Configure the Remote Sensor to be the Selected Sensor of the right Talon */
		_rightConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();
		
		/* Scale the Selected Sensor using a coefficient (Values explained in Constants.java */
		_rightConfig.auxiliaryPID.selectedFeedbackCoefficient = Constants.kTurnTravelUnitsPerRotation / Constants.kPigeonUnitsPerRotation;
		
		/* Set status frame periods */
		_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
		_pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, Constants.kTimeoutMs);
		
		/* Configure neutral deadband */
		_rightConfig.neutralDeadband = Constants.kNeutralDeadband;
		_leftConfig.neutralDeadband = Constants.kNeutralDeadband;		

		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftConfig.peakOutputForward = +1.0;
		_leftConfig.peakOutputReverse = -1.0;
		_rightConfig.peakOutputForward = +1.0;
		_rightConfig.peakOutputReverse = -1.0;

		/* FPID Gains for turn servo */
		_rightConfig.slot1.kP = Constants.kGains_Turning.kP;
		_rightConfig.slot1.kI = Constants.kGains_Turning.kI;
		_rightConfig.slot1.kD = Constants.kGains_Turning.kD;
		_rightConfig.slot1.kF = Constants.kGains_Turning.kF;
		_rightConfig.slot1.integralZone = Constants.kGains_Turning.kIzone;
		_rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_Turning.kPeakOutput;
		_rightConfig.slot1.allowableClosedloopError = 0;
		
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
        int closedLoopTimeMs = 1;
        _rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		
		_rightMaster.configAllSettings(_rightConfig);
		_leftMaster.configAllSettings(_leftConfig);

		

		/* Initialize */
		_firstCall = true;
		_state = false;
		_printCount = 0;
		zeroHeading();
	}
	
	@Override
	public void teleopPeriodic() {
		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		forward = Deadband(forward);
		turn = Deadband(turn);
	
		/* Button processing for state toggle and sensor zeroing */
		getButtons(_currentBtns, _gamepad);
		if(_currentBtns[2] && !_previousBtns[2]){
			_state = !_state; 		// Toggle state
			_firstCall = true;		// Mode change, do first call operation
			_targetAngle = _rightMaster.getSelectedSensorPosition(1);
		}else if (_currentBtns[1] && !_previousBtns[1]) {
			zeroHeading();			//Zero sensors
		}
		/* Store current buttons into previous button array for tracking */
		System.arraycopy(_currentBtns, 0, _previousBtns, 0, Constants.kNumButtonsPlusOne);
				
		/* Select drive mode based on current state */
		if(!_state){
			if (_firstCall)
				System.out.println("This is a basic arcade drive.\n");
			
			_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		}else{
			if (_firstCall) {
				System.out.println("This is Drive Straight using the new Auxillary feature with Pigeon to maintain current angle.\n");
				
				/* Determine which slot affects which PID */
				_rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			}

			/* Configured for percentOutput with Auxiliary PID on Pigeon's Yaw */
			_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.AuxPID, _targetAngle);
			_leftMaster.follow(_rightMaster, FollowerType.AuxOutput1);
			
			/* Print extra Closed Loop information */
			if(_printCount++ > 20){
				System.out.println(	"TargetAng: " + _targetAngle + 
									" CurrentAng: " + _rightMaster.getSelectedSensorPosition(1) + 
									" Error; " + _rightMaster.getClosedLoopError(1));
				_printCount = 0;	// Reset print count
			}
		}
		_firstCall = false;
	}
	
	/** Zero all sensors used. */
	void zeroHeading() {
		_pidgey.setYaw(0, Constants.kTimeoutMs);
		_pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("[Pigeon] All sensors are zeroed.\n");
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
	void getButtons(boolean[] _currentBtns, Joystick gamepad) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			_currentBtns[i] = gamepad.getRawButton(i);
		}
	}
}
