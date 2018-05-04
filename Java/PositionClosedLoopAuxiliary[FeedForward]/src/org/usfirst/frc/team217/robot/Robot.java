/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {
	/** Hardware */
	TalonSRX _leftMaster = new TalonSRX(2);
	TalonSRX _rightMaster = new TalonSRX(1);
	Joystick _gamepad = new Joystick(0);
	
	/** A couple latched values to detect on-press events for buttons and POV */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];
	
	/** Tracking variables */
	boolean bFirstCall = false;
	boolean _state = false;

	@Override
	public void robotInit() {
		/* Don't use this for now */
	}
	
	@Override
	public void teleopInit(){
		/* Disable all motors */
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftMaster.set(ControlMode.PercentOutput,  0);
		
		/* Set neutral modes */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		
		/** Closed loop configuration */
		
		/* Drivetrain's left side Quadrature Encoder */
		_leftMaster.configSelectedFeedbackSensor(	FeedbackDevice.CTRE_MagEncoder_Relative,// Local Feedback Source
													Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
													Constants.kTimeoutMs);					// Configuration Timeout

		/* Supply left Talon's FeedbackSensor to one of the right Talon's remoteSensors */
		_rightMaster.configRemoteFeedbackFilter(_leftMaster.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Source
												Constants.REMOTE_0,							// Remote Number [0, 1]
												Constants.kTimeoutMs);						// Timeout
		
		/* Setup Sum signal to be used for Distance when performing Drive Straight with Pigeon */
		_rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);				// Feedback Device of Remote Talon
		_rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* First sensor marked as 0, used in distance */
		_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													Constants.PID_PRIMARY,
													Constants.kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		_rightMaster.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														Constants.PID_PRIMARY,		// PID Slot of Source 
														Constants.kTimeoutMs);		// Configuration Timeout
		
		/* Configure output and sensor direction */
		_leftMaster.setInverted(false);
		_leftMaster.setSensorPhase(true);
		_rightMaster.setInverted(true);
		_rightMaster.setSensorPhase(true);
		
		//------------ Telemetry-----------------//
		/* Main PID telemetry */
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
		
		/* Speed up the left since we are polling it's sensor */
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

		/* Configure neutral deadband */
		_rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		_leftMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_leftMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

		/* FPID Gains for distance servo */
		_rightMaster.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
		_rightMaster.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
		_rightMaster.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
		_leftMaster.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
		_rightMaster.config_IntegralZone(Constants.kSlot_Distanc, (int)Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
		
		_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Distanc,				//Slot
												Constants.kGains_Distanc.kPeakOutput,	//PercentOut	
												Constants.kTimeoutMs);					//Timeout
		_rightMaster.configAllowableClosedloopError(Constants.kSlot_Distanc, 0, Constants.kTimeoutMs);

			
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

		zeroSensors();
		
		bFirstCall = true;
	}
	
	@Override
	public void teleopPeriodic() {
		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		forward *= 0.75f;
		turn *= 0.75f;
		forward = Deadband(forward);
		turn = Deadband(turn);
	
		/* Button processing for state toggle and sensor zeroing */
		getButtons(btns, _gamepad);
		if(btns[2] && !_btns[2]){
			_state = !_state; 	//Toggle State
			bFirstCall = true;
		}else if (btns[1] && !_btns[1]) {
			zeroSensors();		//Zero Sensors
		}
		CopyButtons(_btns, btns);
				
		if(!_state){
			/* Percent output drive mode for forward and turn */
			one_Axis_PercentOutput(bFirstCall, forward, turn);
		}else{
			/* Percent output drive mode for forward while drive straight using Pigeon */
			one_Axis_Position(bFirstCall, forward, turn);
		}
		
		/* Recreated variables */
		bFirstCall = false;
	}
	
	void one_Axis_PercentOutput(boolean bFirstCall, double joyY, double joyTurn) {
		/* calculate targets from gamepad inputs */
		double left = joyY + joyTurn;
		double rght = joyY - joyTurn;

		if (bFirstCall) {
			System.out.println("This is a basic arcade drive.\n");
		}

		_leftMaster.set(ControlMode.PercentOutput, left);
		_rightMaster.set(ControlMode.PercentOutput, rght);
	}
	
	void one_Axis_Position(boolean bFirstCall, double joyY, double joyTurn) {		
		if (bFirstCall) {
			System.out.println("This is Position Closed Loop with a custom Feed Forward.");
			zeroSensors();
			
			/* Determine which slot affects which PID */
			_rightMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
		}
		
		/* calculate targets from gamepad inputs */
		double target_sensorUnits = joyY * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
		double feedFwdTerm = joyTurn * 0.25; /* how much to add to the close loop output */
		
		_rightMaster.set(ControlMode.Position, target_sensorUnits, DemandType.ArbitraryFeedForward, feedFwdTerm);
		_leftMaster.follow(_rightMaster);
	}
	
	void zeroSensors() {
		_leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		System.out.println("        [Sensors] All sensors are zeroed.\n");
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
	
	/** Store the values of current buttons into a last button state array */
	void CopyButtons(boolean[] destination, boolean[] source) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			destination[i] = source[i];
		}
	}
}
