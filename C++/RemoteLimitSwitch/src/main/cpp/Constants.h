#pragma once

extern struct _Constants {
	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	const static int kTimeoutMs = 30;

	const static int kNumButtonsPlusOne = 10; /* 10 means buttons[1,9] are polled, which is actually 10 buttons */

} Constants; /* use global struct instance so that access semantics match Java */
