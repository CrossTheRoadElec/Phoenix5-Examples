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

START_ROBOT_CLASS(Robot)
