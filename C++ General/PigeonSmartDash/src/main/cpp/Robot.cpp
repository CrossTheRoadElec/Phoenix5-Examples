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
#include <iostream>
#include <memory>
#include <string>

#include "frc/WPIlib.h"
#include "ctre/Phoenix.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	/* forward proto */
	class PlotThread;

	TalonSRX * _talon;
	PigeonIMU * _pidgey;

	PlotThread *_plotThread;

	Robot() {
		_talon = new TalonSRX(2);
		_pidgey = new PigeonIMU(_talon);
		_plotThread = NULL;
	}
	void RobotInit(){
		/* Factory Default all hardware to prevent unexpected behaviour */
		_talon->ConfigFactoryDefault();
		_pidgey->ConfigFactoryDefault();
	}
	void TeleopInit() {
		_plotThread = new PlotThread(this);
	}
	void TeleopPeriodic() { }

	/** quick and dirty threaded plotter */
	class PlotThread {
	public:
		Robot * _robot;
		std::thread * _thrd;
		PlotThread(Robot * robot) {
			this->_robot = robot;
			this->_thrd = new std::thread(&PlotThread::execute, this);
		}

		void execute() {
			/* speed up network tables, this is a test project so eat up all
			 * of the network possible for the purpose of this test.
			 */
			while (true) {
				/* yield for a ms or so - this is not meant to be accurate */
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				/* grab the last signal update from our 1ms frame update */
				double heading = _robot->_pidgey->GetFusedHeading();
				SmartDashboard::PutNumber("hdng", heading);
			}
		}
	};
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
