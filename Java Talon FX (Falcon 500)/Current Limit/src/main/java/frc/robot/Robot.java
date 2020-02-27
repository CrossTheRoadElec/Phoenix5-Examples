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
 * The Current Limit example demonstrates the Talon's ability to perform current limiting.
 * The Current Limit Feature expands across 4 different functions:
 * 1.) configPeakCurrentLimit(), Config current threshold to trigger current limit
 * 2.) configPeakCurrentDuration(), Config duration after peak current to trigger current limit
 * 3.) configContinousCurrentLimit(), Config current to mantain after limit is triggered
 * 4.) enableCurrentLimit(bool enable), Enable/Disable Current Limiting on Talon
 *
 * This example has been configured to hold 10 Amps almost instantly after current
 * exceed peak current limit of 15 Amps.
 * 
 * Controls:
 * Button 1: When held, enable Percent Output. To be used with Left Joystick Y-Axis
 * Button 2: When pressed, peform Postion Closed Loop servo to 0 position
 * Button 5: When pressed, toggle between current limit Enable and Disable.
 * 	Enable/Disable state indcated through prints
 * Left Joystick Y-Axis: Throttle Talon forward and reverse when Button 1 is held
 * 
 * Supported Version:
 * 	- Talon FX: 20.2.3.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class Robot extends TimedRobot {
    /* Hardware */
	TalonFX _tal = new TalonFX(1);
    Joystick _joy = new Joystick(0);
    
    /* Tracking variables */    
	boolean _currentLimEn = true;
	boolean _btn5 = false;

	public void teleopInit() {
        /* Factory Default Hardware to prevent unexpected behaviour */
        _tal.configFactoryDefault();
		
		/**
		 * Configure the current limits that will be used
		 * Stator Current is the current that passes through the motor stators.
		 *  Use stator current limits to limit rotor acceleration/heat production
		 * Supply Current is the current that passes into the controller from the supply
		 *  Use supply current limits to prevent breakers from tripping
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch13_MC.html#current-limit
		 * 
		 *                                                               enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)  */
		_tal.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,      20,                25,                1.0));
        _tal.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,      10,                15,                0.5));

		/* setup a basic closed loop */
		_tal.setNeutralMode(NeutralMode.Brake); // Netural Mode override 
        _tal.configSelectedFeedbackSensor(  TalonFXFeedbackDevice.IntegratedSensor, // Sensor Type 
                                            Constants.PID_PRIMARY,      // PID Index
                                            Constants.kTimeoutMs);      // Config Timeout

		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _tal.setSensorPhase(true);
        
        /* Gains for Postion Closed Loop servo */
		_tal.config_kP(Constants.SLOT_0, Constants.kGains.kP, Constants.kTimeoutMs);
		_tal.config_kI(Constants.SLOT_0, Constants.kGains.kI, Constants.kTimeoutMs);
		_tal.config_kD(Constants.SLOT_0, Constants.kGains.kD, Constants.kTimeoutMs);
		_tal.config_kF(Constants.SLOT_0, Constants.kGains.kF, Constants.kTimeoutMs);
	}

	public void teleopPeriodic() {
		/* Get joystick inputs */
		boolean btn1 = _joy.getRawButton(1);
		boolean btn2 = _joy.getRawButton(2);
		boolean btn5 = _joy.getRawButton(5);
		double stick = -1.0 * _joy.getY();

		if (btn1) {
			/* on button 1 press, manual control with stick */
			_tal.set(ControlMode.PercentOutput, stick);
		} else if (btn2) {
			/* on button 2 press, servo back to zero */
			double targetPos = 0;
			_tal.set(ControlMode.Position, targetPos);
		} else {
			/* otherwise stop output */
			_tal.set(ControlMode.PercentOutput, 0);
		}

		/* on button5 (shoulder button on Logitech gamepad) */
		if (!_btn5 && btn5) {
			/* toggle current limit */
			_currentLimEn = !_currentLimEn;
			/* update Talon current limit */
			//_tal.enableCurrentLimit(_currentLimEn);
			/* print to DS */
			System.out.println("EnableCurrentLimit:"  + _currentLimEn);
		}
		/* save button state for next loop */
		_btn5 = btn5;
	}
}
