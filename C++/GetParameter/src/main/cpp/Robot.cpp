#include <iostream>
#include <string>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	//Create Talon
	TalonSRX * _talon;

	void RobotInit() {
		//Initialize talon with ID 3
		_talon = new TalonSRX(3);
		/* Factory Default all hardware to prevent unexpected behaviour */
		_talon->ConfigFactoryDefault();

	    /* nonzero to block the config until success, zero to skip checking */
    	const int kTimeoutMs = 30;
		//Configure parameters to talon to get later
		_talon->Config_kP(0, 0.2, kTimeoutMs);
		_talon->ConfigForwardSoftLimitThreshold(200, kTimeoutMs);
	}

	void TeleopInit() {
		//Get Parameters from Talon
		std::cout << "Talon kP is: "
				<< _talon->ConfigGetParameter(ParamEnum::eProfileParamSlot_P, 0,
						0) << std::endl << "Talon Forward Soft Limit is: "
				<< _talon->ConfigGetParameter(
						ParamEnum::eForwardSoftLimitThreshold, 0, 0)
				<< std::endl;
	}
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
