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
 * The VelocityClosedLoop_AuxFeedForward example demonstrates the new Talon/Victor auxiliary
 * and remote features to perform complex closed loops. This example has the robot performing 
 * Velocity Closed Loop with an auxiliary feed-forward to request more or less output.
 * 
 * This example uses:
 * - 2x Falcon 500 Integrated Sensors, one on both sides of robot for Primary Closed Loop on Position
 * A Talon/Victor caclulates the distance by taking the sum of both sensors and diving it by 2.
 * 
 * This example has two modes of operation, which can be switched between with Button 2.
 * 1.) Arcade Drive
 * 2.) Velocity Closed Loop with Integrated Sensors and FeedForward
 * 
 * Controls:
 * Button 1: When pressed, zero all sensors. Set Integrated Sensors' positions to 0.
 * Button 2: When pressed, toggle between Arcade Drive and Velocity Closed Loop
 * Left Joystick Y-Axis:
 * 	+ Arcade Drive: Drive robot forward and reverse
 * 	+ Velocity Closed Loop: Servo robot forward and reverse [-500, 500] RPM
 * Right Joystick X-Axis:
 * 	+ Arcade Drive: Turn robot right and left
 * 	+ Velocity Closed Loop: Request more or less output on Closed Loop [-10, 10] %
 * 
 * Gains for Velocity Closed Loop may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon FX: 20.2.3.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
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
	
	/** Latched values to detect on-press events for buttons and POV */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];

	/** Invert Directions for Left and Right */
	TalonFXInvertType _leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	TalonFXInvertType _rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

	/** Config Objects for motor controllers */
	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
	
	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;

	@Override
	public void robotInit() {
		/* Not used in this project */
	}
	
	@Override
	public void teleopInit(){
		/* Disable all motors */
		_rightMaster.set(TalonFXControlMode.PercentOutput, 0);
        _leftMaster.set(TalonFXControlMode.PercentOutput,  0);
		
		/* Set neutral modes */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);

		/* Configure output */
		_leftMaster.setInverted(TalonFXInvertType.CounterClockwise);
		_rightMaster.setInverted(TalonFXInvertType.Clockwise);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _leftMaster.setSensorPhase(true);
        // _rightMaster.setSensorPhase(true);


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
		setRobotDistanceConfigs(_rightInvert, _rightConfig);

		/* FPID for Distance */
		_rightConfig.slot2.kF = Constants.kGains_Velocit.kF;
		_rightConfig.slot2.kP = Constants.kGains_Velocit.kP;
		_rightConfig.slot2.kI = Constants.kGains_Velocit.kI;
		_rightConfig.slot2.kD = Constants.kGains_Velocit.kD;
		_rightConfig.slot2.integralZone = Constants.kGains_Velocit.kIzone;
		_rightConfig.slot2.closedLoopPeakOutput = Constants.kGains_Velocit.kPeakOutput;

		/* false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 *   This is typical when the master is the right Talon FX and using Pigeon
		 * 
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 *   This is typical when the master is the left Talon FX and using Pigeon
		 */
		_rightConfig.auxPIDPolarity = false;


		/* Config the neutral deadband. */
		_leftConfig.neutralDeadband = Constants.kNeutralDeadband;
		_rightConfig.neutralDeadband = Constants.kNeutralDeadband;

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

		/* Motion Magic Configs */
		_rightConfig.motionAcceleration = 2000; //(distance units per 100 ms) per second
		_rightConfig.motionCruiseVelocity = 2000; //distance units per 100 ms



		/* APPLY the config settings */
		_leftMaster.configAllSettings(_leftConfig);
		_rightMaster.configAllSettings(_rightConfig);
		
		/* Set status frame periods to ensure we don't have stale data */
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

		
		/* Initialize */
		_firstCall = true;
		_state = false;
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
			_state = !_state; 	// Toggle state
			_firstCall = true;	// State change, do first call operation
		}else if (btns[1] && !_btns[1]) {
			zeroSensors();		// Zero sensors
		}
		System.arraycopy(btns, 0, _btns, 0, Constants.kNumButtonsPlusOne);
		
		if(!_state){
			if (_firstCall)
				System.out.println("This is Arcade drive.\n");
			
			_leftMaster.set(TalonFXControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			_rightMaster.set(TalonFXControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
			
			/* Uncomment to view velocity native units */
            //System.out.println("Native RPM: " + _rightMaster.getSelectedSensorVelocity());
		}else{
			if (_firstCall) {
				System.out.println("This is Velocity Closed Loop with an Arbitrary Feed Forward.");
				System.out.println("Travel [-500, 500] RPM while having the ability to add a FeedForward with joyX ");
				zeroSensors();
				
				/* Determine which slot affects which PID */
				_rightMaster.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
			}
			
			/* Calculate targets from gamepad inputs */
			double target_RPM = forward * 2000;	// +- 2000 RPM
			double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;	//RPM -> Native units
			double feedFwdTerm = turn * 0.10;	// Percentage added to the close loop output
			
			/* Configured for Velocity Closed Loop on Integrated Sensors' Sum and Arbitrary FeedForward on joyX */
			_rightMaster.set(TalonFXControlMode.Velocity, target_unitsPer100ms, DemandType.ArbitraryFeedForward, feedFwdTerm);
			_leftMaster.follow(_rightMaster);
			
			/* Uncomment to view RPM in Driver Station */
            // double actual_RPM = (_rightMaster.getSelectedSensorVelocity() / (double)Constants.kSensorUnitsPerRotation * 600f);
            // System.out.println("Vel[RPM]: " + actual_RPM + " Pos: " + _rightMaster.getSelectedSensorPosition());
		}
		_firstCall = false;
	}
		
	/* Zero all sensors on Talons */
	void zeroSensors() {
		_leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		System.out.println("[Integrated Sensors] All sensors are zeroed.\n");
	}
	
	/** Deadband 5 percent, used on the gamepad (To be added to Framework?) */
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
}
