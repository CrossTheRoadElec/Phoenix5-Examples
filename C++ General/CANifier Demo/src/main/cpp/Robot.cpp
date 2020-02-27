/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
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

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
