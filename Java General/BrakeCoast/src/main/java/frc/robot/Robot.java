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
 * The BrakeCoast example demonstrates Talon SRX's/Victor SPX's ability to change
 * neutral mode between Brake and Coast.
 * 
 * Neutral mode can be determined by LED state of B/C Button on either Motor Controller (MC).
 * 	- Brake Mode: LED On
 * 	- Coast Mode: LED Off
 * 
 * Two methods to toggle between Brake and Coast
 * 1.) Use setNeutralMode() to perform software override on MC's neutral Mode.
 * 2.) Use Hardware B/C button on MC to toggle neutral mode state. 
 * 	Only works if setNeutralMode() has not been called in current program 
 * 	(software override prevents hardware B/C button from changing MC's neutral Mode);
 * 
 * Controls:
 * Button 1: Toggle between Coast and Brake, 
 * 	- Indicated by print to DriverStation and LED color of B/C LED/Button on MC
 */

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot {
    /* Hardware */
	TalonSRX _tal = new WPI_TalonSRX(1);    // Test Talon
    Joystick _joy = new Joystick(0);    // Test Gamepad  

	/** Save buttons each loop */
    boolean[] _previousBtns = new boolean[10];
    
	/** Desired neutral mode, init value assigned here */
	boolean _brake = true;      // false = coast; true = brake

	/** Constructor sets the neutral mode to start with. */
	public Robot() {
		_tal.configFactoryDefault();
		/* Override neutral mode setting programmatically */
		_tal.setNeutralMode(_brake ? NeutralMode.Brake : NeutralMode.Coast);
		/* instrument to console */
        System.out.println("Brake:" + (_brake ? "Enabled" : "Disabled"));
	}

	/** Every loop, flip brake mode if button 1 when is pressed. */
	public void commonloop() {
		/* Get buttons from gamepad */
		boolean[] currentBtns = new boolean[_previousBtns.length];
		for (int i = 1; i < _previousBtns.length; ++i) { 
			currentBtns[i] = _joy.getRawButton(i); 
		}

		/* Switch between Brake and Coast For Neutral Mode with button press (X-Button) */
		if (currentBtns[1] && !_previousBtns[1]) {
			_brake = !_brake;
			/* Override neutral mode setting programmatically */
			_tal.setNeutralMode(_brake ? NeutralMode.Brake : NeutralMode.Coast);
			/* Instrument to console */
            System.out.println("Brake:" + (_brake ? "Enabled" : "Disabled"));
		}

		/* Save buttons states for on-press detection */
		for (int i = 1; i < _previousBtns.length; ++i) {
			 _previousBtns[i] = currentBtns[i]; 
		}
	}

	public void disabledPeriodic() {
		commonloop();	// just call a "common" loop
	}

	public void teleopPeriodic() {
		commonloop();	// just call a "common" loop
	}
}
