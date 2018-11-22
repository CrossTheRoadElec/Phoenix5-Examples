#pragma once

extern struct _Constants {
	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	const static int kTimeoutMs = 30;

	const static int kNumButtonsPlusOne = 10; /* 10 means buttons[1,9] are polled, which is actually 10 buttons */

	const static int REMOTE_0 = 0;
	const static int REMOTE_1 = 1;

	const static int PID_PRIMARY = 0;
	const static int PID_TURN = 1;

	const static int kForwardSoftLimit_Quad = +4096 * 5; /* 5 rotations assuming CTRE Mag encoder */
	const static int kReverseSoftLimit_Quad = -4096 * 5; /* 5 rotations assuming CTRE Mag encoder */

	const static int kForwardSoftLimit_Pigeon = +341; /* ~15 deg, Pigeon resolution: 8192 => 360 deg */
	const static int kReverseSoftLimit_Pigeon = -341; /* ~15 deg, Pigeon resolution: 8192 => 360 deg */

	const static int kForwardSoftLimit_PWMInput = -1000; /* 4096 => 100% dutyCycle, positive or negative depending on phase. */
	const static int kReverseSoftLimit_PWMInput = -3000; /* 4096 => 100% dutyCycle, negative or negative depending on phase. */

} Constants; /* use global struct instance so that access semantics match Java */
