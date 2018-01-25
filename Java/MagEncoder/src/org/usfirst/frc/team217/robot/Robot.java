/**
 * @brief A quick example on higher resolution plotting a sensor for testing.
 * 
 * Simple example for plotting sensor
 * Sensor is a CTRE Magnetic Encoder plugged into a Talon SRX via Gadgeteer Ribbon Cable.
 * Robot should be propped up on blocks so that the wheels spin free (if testing a drive train sensor).
 * 
 * Talon SRX ...
 * http://www.ctr-electronics.com/talon-srx.html
 * Magnetic Encoder...
 * http://www.ctr-electronics.com/srx-magnetic-encoder.html
 * Cables...
 * http://www.ctr-electronics.com/talon-srx-data-cable-4-pack.html#product_tabs_related_tabbed
 * http://www.ctr-electronics.com/talon-srx-data-cable-kit-new-product.html
 * 
 * SmartDashboard (SD) setup.
 * [1] Open Smartdashboard (I typically (re)select the Dashboard Type in DriverStation if the SD doesn't pop up).
 * [2] Deploy software and enable.
 * [3] Find the text entry in the SD for "vel".  
 * [4] View =>Editable should be checked.
 * [5] Right-click on "vel" label and "Change to..." the Line Plot.  
 * 
 * A few details regarding Smartdashboard in general...
 * [1] Constant data does not render new plot points. So if the signal being measured doesn't change value, the plot stops.
 * Once the signal changes again the plot resumes but the time gap between is truncated in the plot.
 * [2] Changing the window of samples is done by View=>Editable=>Check, then right click-properties on the plot.
 * 		Then change "Buffer Size" in the popup.   I find myself changing this often as I learn more about the signal I am viewing.
 * [3] Zoom features will cause the plot to stop updating and I haven't found a quick way to get the plot to resume plotting.  So 
 * 		I've been avoiding Zoom-In/Zoom-Out for now.
 * [4] Right-click properties on the plot does different things depending on if View=>Editable is checked.
 *  
 * @author Ozrien
 */
package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	/**
	 * Just a follower, could be a Talon.
	 */
	VictorSPX _vic = new VictorSPX(1);
	/**
	 * Master Talon
	 */
	TalonSRX _tal = new TalonSRX(2);
	/**
	 * Simple thread to plot the sensor velocity
	 */
	PlotThread _plotThread;
	/**
	 * joystick or gamepad
	 */
	Joystick _joystick = new Joystick(0);
	
	public void teleopInit() {
		/* Victor will follow Talon */
		_vic.follow(_tal);

		/*
		 * new frame every 1ms, since this is a test project use up as much
		 * bandwidth as possible for the purpose of this test.
		 */
		_tal.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1, 10);
		_tal.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		/* fire the plotter */
		_plotThread = new PlotThread(this);
		new Thread(_plotThread).start();
	}

	public void teleopPeriodic() {
		/*
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
			_tal.set(ControlMode.PercentOutput, 0.25); /* 25 % output */
		else 
			_tal.set(ControlMode.PercentOutput, 0.0);
	}

	/** quick and dirty threaded plotter */
	class PlotThread implements Runnable {
		Robot robot;

		public PlotThread(Robot robot) {
			this.robot = robot;
		}

		public void run() {
			/*
			 * speed up network tables, this is a test project so eat up all of
			 * the network possible for the purpose of this test.
			 */
			// NetworkTable.setUpdateRate(0.010); /* this suggests each time
			// unit is 10ms in the plot */
			while (true) {
				/* yield for a ms or so - this is not meant to be accurate */
				try {
					Thread.sleep(1);
				} catch (Exception e) {
				}
				/* grab the last signal update from our 1ms frame update */
				double velocity = this.robot._tal.getSelectedSensorVelocity(0);
				SmartDashboard.putNumber("vel", velocity);
			}
		}
	}
}
