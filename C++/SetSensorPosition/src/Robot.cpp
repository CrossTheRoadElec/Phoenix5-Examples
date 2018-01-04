#include <iostream>
#include <memory>
#include <string>
#include "ctre/Phoenix.h"
#include "WPILib.h"
#include <unistd.h>


class Robot: public frc::IterativeRobot {
public:
	TalonSRX *_srx = new TalonSRX(3);
	Joystick _joy;
	std::stringstream _work;
	bool _btn1, _btn2, _btn3, _btn4;
	/** simple constructor */
	Robot() : _joy(0), _work(), _btn1(false), _btn2(false), _btn3(false), _btn4(false) 	{	}
	/* everytime we enter disable, reinit*/
	void DisabledInit() {
		_srx->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10); /* MagEncoder meets the requirements for Unit-Scaling */
		_srx->SetStatusFramePeriod(StatusFrame::Status_1_General_, 5, 10); /* Talon will send new frame every 5ms */
	}
	/* every loop */
	void DisabledPeriodic() {
		bool btn1 = _joy.GetRawButton(1);	/* get buttons */
		bool btn2 = _joy.GetRawButton(2);
		bool btn3 = _joy.GetRawButton(3);
		bool btn4 = _joy.GetRawButton(4);

		/* on button unpress => press, change pos register */
		if(!_btn1 && btn1) {			_srx->SetSelectedSensorPosition(10, 0, 0);			_work << "set:10.0" << std::endl;		}
		if(!_btn2 && btn2) {			_srx->SetSelectedSensorPosition(20, 0, 0);			_work << "set:20.0" << std::endl;		}
		if(!_btn3 && btn3) {			_srx->SetSelectedSensorPosition(30, 0, 0);			_work << "set:30.0" << std::endl;		}
		if(!_btn4 && btn4) {			_srx->SetSelectedSensorPosition(40, 0, 0);			_work << "set:40.0" << std::endl;		}

		/* remove this and at most we get one stale print (one loop) */
		usleep(10e3);

		/* call get and serialize what we get */
		double read = _srx->GetSelectedSensorPosition(0);
		_work << "read:" << read<< std::endl;

		/* print any rendered strings, and clear work */
		printf(_work.str().c_str());
		_work.str("");

		_btn1 = btn1; /* save button states */
		_btn2 = btn2;
		_btn3 = btn3;
		_btn4 = btn4;
	}
};

START_ROBOT_CLASS(Robot)
