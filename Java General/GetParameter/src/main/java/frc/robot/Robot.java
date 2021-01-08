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
 * The GetParameter example demonstrates the Phoenix API used to get various parameters,
 * including parameters that don't have get functions. This example sets a couple of 
 * config parameters and retrieves them for a single print.
 * 
 * Controls:
 * None for this example, deploy and enable to see results in Driver Station Prints.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot {
	/* Create a Talon SRX */
	TalonSRX _talon = new WPI_TalonSRX(2);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		/* Factory Default all hardware to prevent unexpected behaviour */
		_talon.configFactoryDefault();
	    /* Nonzero to block the config until success, zero to skip checking */
    	int kTimeoutMs = 30;

        /* Configure Talon parameters, used for retrieval later */
		_talon.config_kP(0, 0.25, kTimeoutMs);
		_talon.configForwardSoftLimitThreshold(200, kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopInit() {
		/* On teleop init, print the configured setting */
		double kP_value = _talon.configGetParameter(ParamEnum.eProfileParamSlot_P, 0, 0);
		double fwdSoftLimitThres = _talon.configGetParameter(ParamEnum.eForwardSoftLimitThreshold, 0, 0);
		System.out.println("P gain is: " + kP_value);
		System.out.println("Software limit forward is " + fwdSoftLimitThres);
	}

	@Override
	public void teleopPeriodic(){
		/* Run forever, do nothing else in program */
	}
}
