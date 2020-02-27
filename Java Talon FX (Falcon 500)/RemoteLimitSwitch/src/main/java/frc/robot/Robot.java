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
 * The RemoteLimitSwitch example demonstrates the new Talon/Victor remote features, which
 * can be used as remote Hardware Limit Switches. The project allows for quick change in
 * limit switch/sensor configuration between:
 * 1.) Disable Limit Switches
 * 2.) Enable Local Limit Switch from feedback connector
 * 3.) Enable Remote Limit Switch from a remote CANifier
 * 4.) Enable Remote Limit Switch from a remote Talon FX
 * The project also allows for enable/disable on hardware limit triggering when sensor is not 
 * present from remote sources. configLimitSwitchDisableNeutralOnLOS() only works for remote 
 * limit switch sensor sources.
 * 
 * This project is a quick overview on possible remote soft limit sources and the
 * ability to enable/disable neutral output when remote limit source can't be found. 
 * 
 * Controls:
 * Current Sensor Configurtions are indicated by print messages in DriverStation
 * 1.) Disable Limit Switches           							Button 1
 * 2.) Enable Local Limit Switch from feedback connector  			Button 2
 * 3.) Enable Remote Limit Switch from a remote CANifier			Button 3
 * 4.) Enable Remote Limit Switch from a remote Talon FX			Button 4
 * 
 * 7.) Disable Neutral Output on loss of remote limit source		Button 6
 * 8.) Enable Neutral Output on loss of remote limit source			Button 8
 * 
 * Left Joystick Y-Axis: Throttle Talon forward and reverse in Percent Output
 * 
 * Supported Version:
 * - Talon FX: 20.2.3.0
 * - CANifier: 20.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Robot extends TimedRobot {
    /* Hardware */
    TalonFX _motorCntrller = new TalonFX(1);	// Victor SPX can be used with remote sensor features.
    CANifier _canifLimits = new CANifier(0);	// Use this CANifier for limit switches
    TalonFX _talonLimits = new TalonFX(2); 	// Use this Talon for limit switches
    Joystick _joy = new Joystick(0);			// Input

	/** A couple latched values to detect on-press events for buttons */
	boolean[] _previousBtns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] _currentBtns = new boolean[Constants.kNumButtonsPlusOne];

	void InitRobot() {
        /* Set robot output to neutral at start */
		_motorCntrller.set(ControlMode.PercentOutput, 0);

		/* Factory Default all hardware to prevent unexpected behaviour */
		_motorCntrller.configFactoryDefault();
		_canifLimits.configFactoryDefault();
		_talonLimits.configFactoryDefault();

		/* Pick directions */
		_motorCntrller.setSensorPhase(false);
		_motorCntrller.setInverted(false);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _motorCntrller.setSensorPhase(true);
	}
	
	void SelectLimitSwitch(int choice) {
		if (choice == 0) {
			/* use feedback connector but disable feature, use-webdash to reenable */
			_motorCntrller.configForwardLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													        LimitSwitchNormal.Disabled,
													        Constants.kTimeoutMs);

			_motorCntrller.configReverseLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													        LimitSwitchNormal.Disabled,
													        Constants.kTimeoutMs);

			System.out.println("Limit Switches disabled.");
		} else if (choice == 1) {
			/* use feedback connector - use three functions */
			_motorCntrller.configForwardLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													        LimitSwitchNormal.NormallyOpen,
													        Constants.kTimeoutMs);

			_motorCntrller.configReverseLimitSwitchSource(	LimitSwitchSource.FeedbackConnector,
													        LimitSwitchNormal.NormallyOpen,
													        Constants.kTimeoutMs);

			System.out.println("Limit Switches locally enabled.");
		} else if (choice == 2) {
			/* use remote CANifier - use four param functions */
			_motorCntrller.configForwardLimitSwitchSource(	RemoteLimitSwitchSource.RemoteCANifier,
													        LimitSwitchNormal.NormallyOpen,
													        _canifLimits.getDeviceID(),
													        Constants.kTimeoutMs);

			_motorCntrller.configReverseLimitSwitchSource(	RemoteLimitSwitchSource.RemoteCANifier,
													        LimitSwitchNormal.NormallyOpen,
													        _canifLimits.getDeviceID(),
													        Constants.kTimeoutMs);

			System.out.println("Remote Limit Switches enabled using CANifier.");
		}  else if (choice == 3) {
			/* use remote Talon - use four param functions */
			_motorCntrller.configForwardLimitSwitchSource(	RemoteLimitSwitchSource.RemoteTalonSRX,
													        LimitSwitchNormal.NormallyOpen,
													        _talonLimits.getDeviceID(),
													        Constants.kTimeoutMs);

			_motorCntrller.configReverseLimitSwitchSource(	RemoteLimitSwitchSource.RemoteTalonSRX,
													        LimitSwitchNormal.NormallyOpen,
													        _talonLimits.getDeviceID(),
													        Constants.kTimeoutMs);

			System.out.println("Remote Limit Switches enabled using another Talon.");
		}
	}
	
	void CommonLoop() {
        /* Gamepad processing */
        getButtons(_currentBtns);               // Update buttons
        double joyForward = -1 * _joy.getY();   // positive stick => forward
        joyForward = deadband(joyForward);      // Deadband stick

        /* Select Hardware Limit Switch Source Configurtion */
		if (_currentBtns[1] && !_previousBtns[1]) { SelectLimitSwitch(0); } // Button 1
		if (_currentBtns[2] && !_previousBtns[2]) { SelectLimitSwitch(1); } // Button 2
		if (_currentBtns[3] && !_previousBtns[3]) { SelectLimitSwitch(2); } // Button 3
        if (_currentBtns[4] && !_previousBtns[4]) { SelectLimitSwitch(3); } // Button 4
        
        /* Enable /Disable Limit Switch Triggering on Loss of Sensor Presense */
		if (_currentBtns[6] && !_previousBtns[6]) {                         // Button 6
			/* Don't neutral motor if remote limit source is not available */
            _motorCntrller.configLimitSwitchDisableNeutralOnLOS(true, Constants.kTimeoutMs);

			System.out.println("Checking disabled for sensor presence");
		}
		if (_currentBtns[8] && !_previousBtns[8]) {                         // Button 8
			/* Neutral motor if remote limit source is not available */
            _motorCntrller.configLimitSwitchDisableNeutralOnLOS(false, Constants.kTimeoutMs);

			System.out.println("Checking enabled for sensor presence");
        }
        /* Copy current buttons into previous buttons for tracking */
        System.arraycopy(_currentBtns, 0, _previousBtns, 0, _currentBtns.length);

		/* Drive Talon in Percent Output with gamepad*/
		_motorCntrller.set(ControlMode.PercentOutput, joyForward);
	}
	
	//------------------------- Loops -------------------------------//
		public void disabledInit() {
        /**
         * Initialize hardware so that sensor phases on set before Teleop. This makes
         * self-test more useful.
         */
			InitRobot();
		}

		public void disabledPeriodic() {
			CommonLoop();
		}

		public void teleopInit() {
			/**
			 * Initialize hardware at start of teleop, just in case Talon was replaced or
			 * field-upgraded during disable. All params are persistent except for status
			 * frame periods.
			 */
			InitRobot();
		}
		
		public void teleopPeriodic() {
			CommonLoop();
		}

		//-------------- Some helpful routines ---------------//
		void getButtons(boolean[] _currentBtns) {
			for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
				_currentBtns[i] = _joy.getRawButton(i);
			}
		}

		double deadband(double value) {
			if (value >= +0.05) {
				return value;
			}
			if (value <= -0.05) {
				return value;
			}
			return 0;
		}
}
