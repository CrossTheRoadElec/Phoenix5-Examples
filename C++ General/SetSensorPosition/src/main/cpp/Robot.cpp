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
 * [1] Enable robot and use gamepad y-axis to drive Talon.
 * [2] If sensor is out of phase, self-test will show the sticky fault.
 * [3] Use button 4 to read the CTRE pulse width and seed the quadrature/relative sensor.
 * [4] Use button 1,2,3 to set the sensor position to constant values.
 */
#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <thread>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "PhysicsSim.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	TalonSRX *_srx = new WPI_TalonSRX(0);
	Joystick *_joy = new Joystick(0);
	std::stringstream _work;
	bool _btn1 = false, _btn2 = false, _btn3 = false, _btn4 = false;
	const bool kInvert = true; /* pick this based on your preference on what positive motor output should spin to */
	const bool kSensorPhase = false; /* pick this so self-test stops reporting sensor-out-of-phase */

	void SimulationInit() {
		PhysicsSim::GetInstance().AddTalonSRX(*_srx, 0.75, 2000, kSensorPhase);
	}
	void SimulationPeriodic() {
		PhysicsSim::GetInstance().Run();
	}

	void RobotInit(){
		/* Factory Default all hardware to prevent unexpected behaviour */
		_srx->ConfigFactoryDefault();
	}
	/* every time we enter disable, reinit*/
	void DisabledInit() {
	    /* nonzero to block the config until success, zero to skip checking */
    	const int kTimeoutMs = 30;
        /* choose quadrature/relative which has a faster update rate */
		_srx->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		_srx->SetStatusFramePeriod(StatusFrame::Status_1_General_, 5, kTimeoutMs); /* Talon will send new frame every 5ms */
		_srx->SetSensorPhase(kSensorPhase);
		_srx->SetInverted(kInvert);
	}
	void DisabledPeriodic() {
		CommonLoop();
	}
	void TeleopPeriodic() {
		CommonLoop();
	}

	/* every loop */
	void CommonLoop() {
		bool btn1 = _joy->GetRawButton(1); /* get buttons */
		bool btn2 = _joy->GetRawButton(2);
		bool btn3 = _joy->GetRawButton(3);
		bool btn4 = _joy->GetRawButton(4);

		/* on button unpress => press, change pos register */
		if (!_btn1 && btn1) {
			_srx->SetSelectedSensorPosition(-10, 0, 0);
			_work << "set:-10.0" << std::endl;
		}
		if (!_btn2 && btn2) {
			_srx->SetSelectedSensorPosition(-20, 0, 0);
			_work << "set:-20.0" << std::endl;
		}
		if (!_btn3 && btn3) {
			_srx->SetSelectedSensorPosition(+30, 0, 0);
			_work << "set:+30.0" << std::endl;
		}
		if (!_btn4 && btn4) {
			/* read the mag encoder sensor out */
			int read = (int) _srx->GetSensorCollection().GetPulseWidthPosition();
			/* flip pulse width to match selected sensor.  */
			if (kSensorPhase)
				read *= -1;
			if (kInvert)
				read *= -1;
			/* throw out the overflows, CTRE Encoder is 4096 units per rotation => 12 bitmask (0xFFF) */
			read = read & 0xFFF;
			/* set the value back with no overflows */
			_srx->SetSelectedSensorPosition(read, 0, 0);
			_work << "set:" << read << std::endl;
		}

		/* remove this and at most we get one stale print (one loop) */
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		/* call get and serialize what we get */
		double read = _srx->GetSelectedSensorPosition(0);
		_work << "read:" << read << std::endl;

		/* print any rendered strings, and clear work */
		printf(_work.str().c_str());
		_work.str("");

		/* cache values for next loop comparisons */
		_btn1 = btn1;
		_btn2 = btn2;
		_btn3 = btn3;
		_btn4 = btn4;

		_srx->Set(ControlMode::PercentOutput, -1 * _joy->GetY());
	}
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
