#include "WPILIB.h"

#include "ctrlib/HsvToRgb.h"
#include "ctrlib/RCRadio3Ch.h"
#include "ctrlib/canifier.h"

class Robot: public frc::IterativeRobot {
public:
	CTRE::CANifier* _canifier;
	CTRE::RCRadio3Ch* _controller;
	CTRE::HsvToRgb* _hsvToRgb;

	float _theta;
	float _saturation;
	float forward;
	float turn;
	bool toggleSwitch;
	float _value = 0.05f;

	void RobotInit() {
		_canifier = new CTRE::CANifier(0);
		_controller = new CTRE::RCRadio3Ch(_canifier);
		_hsvToRgb = new CTRE::HsvToRgb();
	}

	void TeleopPeriodic() {
		//Update values from Futaba controller
		_controller->Process();

		if(_controller->CurrentStatus == CTRE::RCRadio3Ch::Status::Okay)
		{
			//Grab signals from the Futaba controller
			forward = _controller->GetDutyCyclePerc(CTRE::RCRadio3Ch::Channel2);
			turn = _controller->GetDutyCyclePerc(CTRE::RCRadio3Ch::Channel1);
			toggleSwitch = _controller->GetSwitchValue(CTRE::RCRadio3Ch::Channel3);

			//Print values within SmartDashboard
			printf("Forward reading: %f \t", forward);
			printf("Backward reading: %f \t", turn);
			printf("State: \t");
			printf(toggleSwitch ? "true\n" : "false\n");
		}
		else
		{
			printf("Controller is not connected\n");
		}


		if(toggleSwitch == true)
		{
			//Controlled by PWM signals received by CANifier
			float f = forward;
			float t = turn;
			_theta = atan2(f,t) * 180/3.1415926535897;
			_saturation = sqrt(f*f + t*t);
		}
		else if (toggleSwitch == false)
		{
			//Cycle through rim of HSV color wheel
			_saturation = 1;
			_theta += 1;
		}
		if(_theta >= 360){_theta = 0;}
		float r,g,b;
		_hsvToRgb->Convert(_theta,_saturation, _value, &r, &g, &b);
		_canifier->SetLEDOutput(r, CTRE::CANifier::LEDChannelA);
		_canifier->SetLEDOutput(g, CTRE::CANifier::LEDChannelB);
		_canifier->SetLEDOutput(b, CTRE::CANifier::LEDChannelC);
	}
};

START_ROBOT_CLASS(Robot)
