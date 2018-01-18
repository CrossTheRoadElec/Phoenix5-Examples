/**
 * IMPORTANT: This example requires the version of the SDK from the
 * Installer version 5.0.3.2 or higher.  There were several changes
 * and additions to multiple files in the SDK, and these are required
 * for this example
 *
 * Anything under the "Framework" folder are tentative changes later to be into the Phoenix Framework.
 *
 * Anything under "Tasks" are typical examples tasks done on a robot platform.
 */

#include "WPILIB.h"

#include "Platform/Platform.h"

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() {
	}

	void TeleopInit() {
		/* Add each task to the concurrent scheduler */
		for (auto ILoopable : Tasks::FullList) {
			Schedulers::PeriodicTasks->Add(ILoopable);
		}
	}

	void TeleopPeriodic() {
		/* Run forever */

		/* Process the scheduler to process each task/loop */
		Schedulers::PeriodicTasks->Process();
	}
};

START_ROBOT_CLASS(Robot)
