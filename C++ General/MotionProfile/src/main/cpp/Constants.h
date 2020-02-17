#pragma once

enum Constants {

	/** which Talon on CANBus*/
	kTalonID = 0,

	/* example Victor SPX follower */
	kVictorFollower = 0,

	/**
	 * How many sensor units per rotation.
	 * Using CTRE Magnetic Encoder.
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
	kSensorUnitsPerRotation = 4096,

	/**
	 * Which PID slot to pull gains from.  Starting 2018, you can choose 
	 * from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based configuration.
	 */
	kSlotIdx = 0,

	/** 
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.  
	 * For now we just want the primary one.
	 */
	kPIDLoopIdx = 0,
	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	kTimeoutMs = 30,

	/**
	 * Base trajectory period to add to each individual 
	 * trajectory point's unique duration.  This can be set
	 * to any value within [0,255]ms.
	 */
	kBaseTrajPeriodMs = 0,

	/**
	 * Motor deadband, set to 1%.
	 */
	kNeutralDeadbandPercent = 1,
};
