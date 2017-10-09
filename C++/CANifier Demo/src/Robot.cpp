/** This example uses the latest build of CTRE's Library which can be installed
 * through CTRE's Toolsuite Installer (5.0.X.X).
 *
 * Anything under the "Framework" folder are tenative changes later to be into the Phoenix Framework.
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
		for(auto ILoopable : Tasks::FullList){
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
