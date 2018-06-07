/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/********* Version *********/
/* TalonSRX: 3.9
 * VictorSPX: 3.9
 * Pigeon IMU: 0.41
 * Phoenix Framework: 5.6.0 
 */

package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {
	/** Hardware */
	TalonSRX _leftMaster = new TalonSRX(2);
	TalonSRX _rightMaster = new TalonSRX(1);
	PigeonIMU _pidgey = new PigeonIMU(3);
	Joystick _gamepad = new Joystick(0);
	
	/** Latched values to detect on-press events for buttons and POV */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];
	int _pov = 0;

	
	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	
	/** Motion profile example manager*/
	MotionProfileExample  _motProfExample = new MotionProfileExample(_rightMaster);
	
	/** Used to help control the Motion Profile Close Looping */
	enum ButtonEvent {
		ButtonOff, ButtonOffToOn, ButtonOn, ButtonOnToOff;
	}

	@Override
	public void robotInit() {
		/* Not in use */
	}
	
	@Override
	public void teleopInit(){
		/* Disable all motors */
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftMaster.set(ControlMode.PercentOutput,  0);
		
		/* Set neutral modes */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		
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
		
		/* Configure the Pigeon IMU to the other Remote Slot on the Right Talon */
		_rightMaster.configRemoteFeedbackFilter(_pidgey.getDeviceID(),
												RemoteSensorSource.Pigeon_Yaw,
												Constants.REMOTE_1,	
												Constants.kTimeoutMs);
		
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
		
		/* Configure Remote Slot 1 [Pigeon IMU's Yaw] to be used for Auxiliary PID Index */
		_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1,
													Constants.PID_TURN,
													Constants.kTimeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient (Configured for 3600 units of resolution) */
		_rightMaster.configSelectedFeedbackCoefficient(	Constants.kTurnTravelUnitsPerRotation / Constants.kPigeonUnitsPerRotation,
														Constants.PID_TURN,
														Constants.kTimeoutMs);
		
		/* Configure output and sensor direction */
		_leftMaster.setInverted(false);
		_leftMaster.setSensorPhase(true);
		_rightMaster.setInverted(true);
		_rightMaster.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

		/* Configure neutral deadband */
		_rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		_leftMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		
		/* Motion Magic Configurations */
		_rightMaster.configMotionAcceleration(2000, Constants.kTimeoutMs);
		_rightMaster.configMotionCruiseVelocity(2000, Constants.kTimeoutMs);

		/* Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_leftMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

		/* FPID Gains for Motion Profile servo */
		_rightMaster.config_kP(Constants.kSlot_MotProf, Constants.kGains_MotProf.kP, Constants.kTimeoutMs);
		_rightMaster.config_kI(Constants.kSlot_MotProf, Constants.kGains_MotProf.kI, Constants.kTimeoutMs);
		_rightMaster.config_kD(Constants.kSlot_MotProf, Constants.kGains_MotProf.kD, Constants.kTimeoutMs);
		_rightMaster.config_kF(Constants.kSlot_MotProf, Constants.kGains_MotProf.kF, Constants.kTimeoutMs);
		_rightMaster.config_IntegralZone(Constants.kSlot_MotProf, (int)Constants.kGains_MotProf.kIzone, Constants.kTimeoutMs);
		_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_MotProf, Constants.kGains_MotProf.kPeakOutput, Constants.kTimeoutMs);
		_rightMaster.configAllowableClosedloopError(Constants.kSlot_MotProf, 0, Constants.kTimeoutMs);

		/* FPID Gains for turn servo */
		_rightMaster.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		_rightMaster.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		_rightMaster.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		_rightMaster.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		_rightMaster.config_IntegralZone(Constants.kSlot_Turning, (int)Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
		_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
		_rightMaster.configAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);

		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rightMaster.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 0, Constants.kTimeoutMs);
		_rightMaster.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 1, Constants.kTimeoutMs);

		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		_rightMaster.configAuxPIDPolarity(false, Constants.kTimeoutMs);

		/* Initialize */
		_firstCall = true;
		_state = false;
		zeroSensors();
	}
	
	@Override
	public void teleopPeriodic() {
		/* Temp for the first calls or button events */
		ButtonEvent bExecuteAction = ButtonEvent.ButtonOff;

		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		forward = Deadband(forward);
		turn = Deadband(turn);
		
		
		getButtons(btns, _gamepad);	
		if (btns[4] && !_btns[4]) {
			_state =  !_state;
			_firstCall = true;
		}else if (btns[1] && !_btns[1]) {
			/* if button1 is just pressed */
			zeroSensors();
		}

		/* look for new button presses */
		if (btns[2] && !_btns[2])
			bExecuteAction = ButtonEvent.ButtonOffToOn;
		else if (!btns[2] && _btns[2]) 
			bExecuteAction = ButtonEvent.ButtonOnToOff;
		else if (btns[2]) 
			bExecuteAction = ButtonEvent.ButtonOn;
 		else 
			bExecuteAction = ButtonEvent.ButtonOff;
		System.arraycopy(btns, 0, _btns, 0, Constants.kNumButtonsPlusOne);
		
		if(!_state){
			if (_firstCall)
				System.out.println("This is a basic arcade drive.\n");
			
			_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		}else{
			/* Calculate targets from gamepad inputs */
			boolean bMoveForward = (forward >= 0) ? true : false;
			double finalHeading_units = Constants.kTurnTravelUnitsPerRotation * forward * -1.0; /* positive right stick => negative heading target (turn to right) */

			if (_firstCall) {
				System.out.println("This is Motion Profile Auxiliary, also known as MotionMagicArc using the Pigeon for turn");
				System.out.println("Press Button 2 (A-Button) to fire the profile.");
				neutralMotors("Target not set yet.\n");

				/* Slots are selected in the profile, not via selectProfileSlot() */

			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				/* Do Nothing in this state */
			} else if (bExecuteAction == ButtonEvent.ButtonOffToOn) {
				neutralMotors("Button let go\n");
				zeroSensors();
				_motProfExample.reset();
				_motProfExample.start(finalHeading_units, bMoveForward);
			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				/* Configured for Motion Profile on Quad Encoders' Sum and Auxiliary PID on Pigeon IMU's Yaw */
				_rightMaster.set(ControlMode.MotionProfileArc, _motProfExample.getSetValue().value);
				_leftMaster.follow(_rightMaster, FollowerType.AuxOutput1);
			}
			/* Call this periodically, and catch the output.  Only apply it if user wants to run MP. */
			_motProfExample.control();
		}
		
		_firstCall = false;
	}
	
	/* Zeroes Quad Encoders on Talons */
	void zeroSensors() {
		_leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		System.out.println("[Sensors] All sensors are zeroed.\n");
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
}
