/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	TalonSRX[] talons;
	PollingThread[] threads;

	private class PollingThread extends Thread {
		TalonSRX ref;
		boolean run = true;

		public PollingThread(TalonSRX tal) {
			ref = tal;
		}

		public void run() {
			try {
				while (run) {
					ref.getSelectedSensorVelocity();
				}
			} catch (Exception e) {

			}
		}

		public void cancel() { run = false; }
	}

	final int TALON_COUNT = 12;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		System.out.println(String.format("0x%08X", com.ctre.phoenix.unmanaged.Unmanaged.getPhoenixVersion()));

	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {


		/*
		talons = new TalonSRX[TALON_COUNT];
		threads = new PollingThread[TALON_COUNT];

		for (int i = 0; i < TALON_COUNT; i++) {
			talons[i] = new TalonSRX(i);
			threads[i] = new PollingThread(talons[i]);

			threads[i].start();
		}
		*/
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
	}

	VictorSPX tal = new VictorSPX(1);
	@Override
	public void teleopInit() {
		/*for(int i = 0; i < TALON_COUNT; i++)
		{
			threads[i].cancel();
		}
		*/
		tal.setStatusFramePeriod(StatusFrame.Status_1_General, 1);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	int count =0;
	@Override
	public void teleopPeriodic() {
		if(count++ > 10)
		{
			count = 0;
			System.out.println(tal.getStatusFramePeriod(StatusFrame.Status_1_General));
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
