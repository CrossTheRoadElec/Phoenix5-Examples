package frc.robot;

public class Constants {
	/** 
	 * ID of Talon on CAN Bus 
	 */
	public static final int kTalonID = 0;

	/**
	 * How many sensor units per rotation. Using Falcon Integrated Sensor.
	 * 
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
	public static final double kSensorUnitsPerRotation = 2048;

	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;
	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/**
	 * Base trajectory period to add to each individual trajectory point's
	 * unique duration. This can be set to any value within [0,255]ms.
	 */
	public static final int kBaseTrajPeriodMs = 0;

	/**
	 * Motor deadband, set to 1%.
	 */
	public static final double kNeutralDeadband = 0.01;

    /* Gains used in Position Closed Loop, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    static final Gains kGains = new Gains(0.1, 0.0, 0.0, 1023.0/7200.0, 0, 1.0);
}
