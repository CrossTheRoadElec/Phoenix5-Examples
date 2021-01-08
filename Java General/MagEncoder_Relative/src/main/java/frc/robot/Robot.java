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
 * Simple example for plotting sensor
 * Sensor is a CTRE Magnetic Encoder plugged into a Talon SRX via Gadgeteer Ribbon Cable.
 * Robot should be propped up on blocks so that the wheels spin free (if testing a drive train sensor).
 * 
 * Talon SRX: 			http://www.ctr-electronics.com/talon-srx.html
 * Magnetic Encoder: 	http://www.ctr-electronics.com/srx-magnetic-encoder.html
 * Cables: 				http://www.ctr-electronics.com/talon-srx-data-cable-4-pack.html
 * 						http://www.ctr-electronics.com/talon-srx-data-cable-kit-new-product.html
 * 
 * SmartDashboard (SD) setup.
 * [1] Open Smartdashboard (I typically (re)select the Dashboard Type in DriverStation if the SD doesn't pop up).
 * [2] Deploy software and enable.
 * [3] Find the text entry in the SD for "vel".  
 * [4] View =>Editable should be checked.
 * [5] Right-click on "vel" label and "Change to..." the Line Plot.  
 * 
 * A few details regarding Smartdashboard in general...
 * [1] Constant data does not render new plot points. So if the signal being measured doesn't 
 * 	change value, the plot stops. Once the signal changes again the plot resumes but the time 
 * 	gap between is truncated in the plot.
 * 
 * [2] Changing the window of samples is done by View=>Editable=>Check, then right 
 * 	click-properties on the plot. Then change "Buffer Size" in the popup.I find myself 
 * 	changing this often as I learn more about the signal I am viewing.
 * 
 * [3] Zoom features will cause the plot to stop updating and I haven't found a quick way to
 * 	get the plot to resume plotting.  So I've been avoiding Zoom-In/Zoom-Out for now.
 * 
 * [4] Right-click properties on the plot does different things depending on if 
 * 	View=>Editable is checked.
 *  
 * Controls:
 * Button 1: When held, apply 25 percent output forward on Talon SRX. 
 * 	Victor SPX will follow due to configuration.
 * 
 * Supported Version:
 * - Talon SRX: 4.0
 * - Victor SPX: 4.0
 * - Pigeon IMU: 4.0
 * - CANifier: 4.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.robot.sim.PhysicsSim;

public class Robot extends TimedRobot {
	/** Hardware */
	VictorSPX _vic = new WPI_VictorSPX(1);		// Follower MC, Could be a victor
	TalonSRX _tal = new WPI_TalonSRX(2);		// Master MC, Talon SRX for Mag Encoder
	Joystick _joystick = new Joystick(0);	// Joystick for project control

	/* Simple thread to plot sensor velocity */
	PlotThread _plotThread;

	public void simulationInit() {
		PhysicsSim.getInstance().addTalonSRX(_tal, 0.75, 4000);
		PhysicsSim.getInstance().addVictorSPX(_vic);
	}
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}

	public void teleopInit() {
		/* Factory default hardware to prevent unexpected behavior */
		_vic.configFactoryDefault();
		_tal.configFactoryDefault();

		/* Victor will follow Talon */
		_vic.follow(_tal);

		/* New frame every 1ms, since this is a test project use up as much
		 * bandwidth as possible for the purpose of this test.
		 */
		_tal.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
		_tal.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

		/* Fire the plotter */
		_plotThread = new PlotThread(this);
		new Thread(_plotThread).start();
	}

	public void teleopPeriodic() {
		/**
		 * If there is mechanical deflection, eccentricity, or damage in the
		 * sensor it should be revealed in the plot.
		 * 
		 * For example, an optical encoder with a partially damaged ring will
		 * reveal a periodic dip in the sensed velocity synchronous with each
		 * rotation.
		 * 
		 * This can also be wired to a gamepad to test velocity sweeping.
		 */
		if (_joystick.getRawButton(1))
			_tal.set(ControlMode.PercentOutput, 0.25);	// 25% Output
		else 
			_tal.set(ControlMode.PercentOutput, 0.0);	// 0% Output
		
		System.out.printf("Output: %.4f\n", _tal.getMotorOutputPercent());
	}

	/** 
	 * Quick and dirty threaded plotter class
	 */
	class PlotThread implements Runnable {
		Robot robot;

		public PlotThread(Robot robot) {
			this.robot = robot;
		}

		public void run() {
			/**
			 * Speed up network tables, this is a test project so eat up all of
			 * the network possible for the purpose of this test.
			 */

			while (true) {
				/* Yield for a Ms or so - this is not meant to be accurate */
				try {
					Thread.sleep(1);
				} catch (Exception e) {
					/* Do Nothing */
				}

				/* Grab the latest signal update from our 1ms frame update */
				double velocity = this.robot._tal.getSelectedSensorVelocity(0);
				SmartDashboard.putNumber("vel", velocity);
			}
		}
	}
}
