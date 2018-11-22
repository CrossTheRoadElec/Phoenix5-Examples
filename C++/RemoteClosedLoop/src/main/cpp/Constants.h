#pragma once

struct Gains {
	double kP;
	double kI;
	double kD;
	double kF;
	double kIzone;
	double kPeakOutput;
};

extern struct _Constants {
	/* ---- Flat constants, you should not need to change these ------ */
	const static int REMOTE_0 = 0;
	const static int REMOTE_1 = 1;
	const static int PID_PRIMARY = 0;
	const static int PID_TURN = 1;
	const static int SLOT_0 = 0;
	const static int SLOT_1 = 1;
	const static int SLOT_2 = 2;
	const static int SLOT_3 = 3;

	/**
	 * How many joystick buttons to poll.
	 * 10 means buttons[1,9] are polled, which is actually 9 buttons
	 */
	const static int kNumButtonsPlusOne = 10;

	/**
	 * How many sensor units per rotation.
	 * Using CTRE Magnetic Encoder.
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
	const static int kSensorUnitsPerRotation = 4096;

	constexpr static double kRotationsToTravel = 6;

	/**
	 * How to measure robot heading. 0 for Difference between left and right quad encoder. 1 for Pigeon IMU.
	 */
	const int kHeadingSensorChoice = 0;

	/**
	 * Empirically measure what the difference between encoders per 360'
	 * Drive the robot in clockwise rotations and measure the units per rotation.
	 * Drive the robot in counter clockwise rotations and measure the units per rotation.
	 * Take the average of the two.
	 */
	const static int kEncoderUnitsPerRotation = 51711.0;

	/**
	 * This is a property of the Pigeon IMU, and should not be changed.
	 */
	const static int kPigeonUnitsPerRotation = 8192.0;


	/**
	 * Using the config feature, scale units to 3600 per rotation.
	 * This is nice as it keeps 0.1 deg resolution, and is fairly intuitive.
	 */
	constexpr static double kTurnTravelUnitsPerRotation = 3600;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	const static int kTimeoutMs = 30;

	/**
	 * Base trajectory period to add to each individual 
	 * trajectory point's unique duration.  This can be set
	 * to any value within [0,255]ms.
	 */
	const static int kBaseTrajPeriodMs = 0;

	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	constexpr static double kNeutralDeadband = 0.001;

	//                                         kP   kI   kD   kF              Iz    PeakOut
	constexpr static Gains kGains_Distanc = { 0.1, 0.0,  0.0, 0.0,            100,  0.50 };
	constexpr static Gains kGains_Turning = { 2.0, 0.0,  4.0, 0.0,            200,  1.00 };
	constexpr static Gains kGains_Velocit = { 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 }; /* measured 6800 velocity units at full motor output */
	constexpr static Gains kGains_MotProf = { 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 }; /* measured 6800 velocity units at full motor output */

	const static int kSlot_Distanc = SLOT_0;
	const static int kSlot_Turning = SLOT_1;
	const static int kSlot_Velocit = SLOT_2;
	const static int kSlot_MotProf = SLOT_3;

} Constants; /* use global struct instance so that access semantics match Java */
