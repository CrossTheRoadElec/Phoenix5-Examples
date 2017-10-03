#include "WPILIB.h"

#include "ctrlib/MotorControl/CANTalon.h"
#include "ctrlib/Drive/Mecanum.h"
#include "ctrlib/Utilities.h"
#include "ctrlib/RCRadio3Ch.h"
#include "ctrlib/HsvToRgb.h"
#include "ctrlib/canifier.h"
#include "ctrlib/ctre.h"

class Robot: public frc::IterativeRobot {
public:
	//Create global objects
	CTRE::MotorControl::CANTalon* _leftFront;
	CTRE::MotorControl::CANTalon* _rightFront;
	CTRE::MotorControl::CANTalon* _leftRear;
	CTRE::MotorControl::CANTalon* _rightRear;
	CTRE::Drive::Mecanum* _drivetrain;
	CTRE::RCRadio3Ch* _futaController;
	CTRE::HsvToRgb* _hsvToRgb;
	CTRE::CANifier* _canifier;

	frc::Joystick* _gamepad;

	//Variables for tracking various values
	float _theta, _saturation, _value = 0.5;
	float forward, turn;
	bool toggle;
	int flashValue = 0;
	bool On = false;

	void RobotInit() {
		_hsvToRgb = new CTRE::HsvToRgb();
		_canifier = new CTRE::CANifier(0);
		_futaController = new CTRE::RCRadio3Ch(_canifier);
		_leftFront = new CTRE::MotorControl::CANTalon(1);
		_leftRear = new CTRE::MotorControl::CANTalon(2);
		_rightRear = new CTRE::MotorControl::CANTalon(3);
		_rightFront = new CTRE::MotorControl::CANTalon(4);
		_drivetrain = new CTRE::Drive::Mecanum(_leftFront, _leftRear, _rightFront, _rightRear);
		_drivetrain->SetVoltageRampRate(10);
	}

	void TeleopPeriodic() {
		/* Update Futaba controller values continuously */
		_futaController->Process();
		/* Acknowledge user */
		//_leftFront->SetControlMode(CTRE::MotorControl::ControlMode::BasicControlMode::kPercentVbusBasic);
		printf("Program is working\n");

		/* If Futaba controller is present, drive */
		if (_futaController->CurrentStatus == CTRE::RCRadio3Ch::Status::Okay) {

			forward = _futaController->GetDutyCyclePerc(CTRE::RCRadio3Ch::Channel1);
			turn = _futaController->GetDutyCyclePerc(CTRE::RCRadio3Ch::Channel2);
			toggle = _futaController->GetSwitchValue(CTRE::RCRadio3Ch::Channel3);

			CTRE::Utilities::Deadband(forward, 0.15);
			CTRE::Utilities::Deadband(turn, 0.15);
			_drivetrain->Set(CTRE::Drive::Styles::Basic::PercentOutputBasic, forward,
					turn);

			if (toggle == true) {
				float f = forward;
				float t = turn;
				_theta = atan2(f, t) * 180 / 3.1415926535897;
				_saturation = sqrt(f * f + t * t);
			} else if (toggle == false) {
				_saturation = 1;
				_theta += 1;
			}

			/* Handle wrap-around */
			if (_theta >= 360)
				_theta = 0;

			/* Color based on toggle-switch position and input values */
			float r, g, b;
			_hsvToRgb->Convert(_theta, _saturation, _value, &r, &g, &b);
			_canifier->SetLEDOutput(r, CTRE::CANifier::LEDChannelA);
			_canifier->SetLEDOutput(g, CTRE::CANifier::LEDChannelB);
			_canifier->SetLEDOutput(b, CTRE::CANifier::LEDChannelC);
		} else {	//Controller not found
			flashValue++;
			if (flashValue >= 40) {
				flashValue = 0;		//Reset the flashing
				On = !On;			//Invert flashing state
			}
			/* Blink according to on state affected by the timer above */
			if (On == true) {
				_canifier->SetLEDOutput(255, CTRE::CANifier::LEDChannelA);
				_canifier->SetLEDOutput(255, CTRE::CANifier::LEDChannelB);
				_canifier->SetLEDOutput(255, CTRE::CANifier::LEDChannelC);
			} else {
				_canifier->SetLEDOutput(0, CTRE::CANifier::LEDChannelA);
				_canifier->SetLEDOutput(0, CTRE::CANifier::LEDChannelB);
				_canifier->SetLEDOutput(0, CTRE::CANifier::LEDChannelC);
			}
		}
	}
};

START_ROBOT_CLASS(Robot)
