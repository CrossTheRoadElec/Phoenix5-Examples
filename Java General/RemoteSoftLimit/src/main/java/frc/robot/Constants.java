/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

public class Constants {
	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	public final static int kTimeoutMs = 30;

	/* 10 means buttons[1,9] are polled, which is actually 10 buttons */
	public final static int kNumButtonsPlusOne = 10; 

	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;

	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;

	public final static int kForwardSoftLimit_Quad = +4096 * 5; /* 5 rotations assuming CTRE Mag encoder */
	public final static int kReverseSoftLimit_Quad = -4096 * 5; /* 5 rotations assuming CTRE Mag encoder */

	public final static int kForwardSoftLimit_Pigeon = +341; /* ~15 deg, Pigeon resolution: 8192 => 360 deg */
	public final static int kReverseSoftLimit_Pigeon = -341; /* ~15 deg, Pigeon resolution: 8192 => 360 deg */

	public final static int kForwardSoftLimit_PWMInput = -1000; /* 4096 => 100% dutyCycle, positive or negative depending on phase. */
	public final static int kReverseSoftLimit_PWMInput = -3000; /* 4096 => 100% dutyCycle, positive or negative depending on phase. */
}
