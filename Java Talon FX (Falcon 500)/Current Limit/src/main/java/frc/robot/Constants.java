/**
 * Simple class containing constants used throughout project
 */
package frc.robot;

class Constants{
    /* Current threshold to trigger current limit */
    static final int kPeakCurrentAmps = 15;
    
    /* Duration after current exceed Peak Current to trigger current limit */
    static final int kPeakTimeMs = 0;

    /* Current to mantain once current limit has been triggered */
    static final int kContinCurrentAmps = 10;

    /**
     * Timeout value generally used in parameter configs
     * Non-zero to block the config until success, zero to skip checking 
     */
    static final int kTimeoutMs = 30;

    /**
     * Gains used in Position Closed Loop, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    static final Gains kGains = new Gains(2.0, 0.0, 0.0, 0.0, 0, 1.0);

    /** ---- Flat constants, you should not need to change these ---- */
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/* Firmware currently supports slots [0, 3] and can be used for either PID set */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
}