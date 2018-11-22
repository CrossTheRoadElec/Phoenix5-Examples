/**
 * IMPORTANT: This example requires the version of the SDK from the
 * Installer version 5.0.3.2 or higher. There were several changes
 * and additions to multiple files in the SDK, and these are required
 * for this example
 *
 * Anything under "Tasks" are typical examples tasks done on a robot platform.
 */


#include "frc/WPILib.h"
#include "Platform/Platform.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	void RobotInit() {
		/* Do nothing here */
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
