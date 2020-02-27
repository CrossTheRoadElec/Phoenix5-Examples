package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Quick and dirty threaded plotter class
 */
class PlotThread implements Runnable {

	private TalonSRX _talon;
	private Thread _thread;

	/* some temps for plotting */
	double sen_pos_drv, sen_vel_drv, trgt_pos_drv, trgt_vel_drv, trgt_arbF_drv;
	double sen_pos_turn, trgt_pos_turn, trgt_vel_turn, trgt_arbF_turn;

	public PlotThread(TalonSRX talon)
	{
		_talon = talon;

		_thread= new Thread(this);
		_thread.start();
	}
	
	public void run() {
		/**
		 * Speed up network tables, this is a test project so eat up all of the network
		 * possible for the purpose of this test.
		 */

		while (true) {
			/* Yield for a Ms or so - this is not meant to be accurate */
			try {
				Thread.sleep(5);
			} catch (Exception e) {
				/* Do Nothing */
			}

			ControlMode mode = _talon.getControlMode();


			/* Grab the latest signal update from our 1ms frame update */
			sen_pos_drv = _talon.getSelectedSensorPosition(0);
			sen_vel_drv = _talon.getSelectedSensorVelocity(0);

			sen_pos_turn = _talon.getSelectedSensorPosition(1);

			if (mode == ControlMode.MotionProfileArc) {
				trgt_pos_drv = _talon.getActiveTrajectoryPosition(0);
				trgt_vel_drv = _talon.getActiveTrajectoryVelocity(0);
				trgt_arbF_drv = _talon.getActiveTrajectoryArbFeedFwd(0);
			}
			
			if (mode == ControlMode.MotionProfileArc) {
				trgt_pos_turn = _talon.getActiveTrajectoryPosition(1);
				trgt_vel_turn = _talon.getActiveTrajectoryVelocity(1);
				trgt_arbF_turn = _talon.getActiveTrajectoryArbFeedFwd(1);
			}

			SmartDashboard.putNumber("sen_pos_drv", sen_pos_drv);
			SmartDashboard.putNumber("sen_vel_drv", sen_vel_drv);
			SmartDashboard.putNumber("trgt_pos_drv", trgt_pos_drv);
			SmartDashboard.putNumber("trgt_vel_drv", trgt_vel_drv);
			SmartDashboard.putNumber("trgt_arbF_drv", trgt_arbF_drv);

			SmartDashboard.putNumber("sen_pos_turn", sen_pos_turn);
			SmartDashboard.putNumber("trgt_pos_turn", trgt_pos_turn);
			SmartDashboard.putNumber("trgt_vel_turn", trgt_vel_turn);
			SmartDashboard.putNumber("trgt_arbF_turn", trgt_arbF_turn);
		}
	}
}