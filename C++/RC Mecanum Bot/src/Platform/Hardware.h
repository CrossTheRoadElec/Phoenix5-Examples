#pragma once

#include "WPILib.h"
#include "ctrlib/MotorControl/CANTalon.h"
#include "ctrlib/Drive/Mecanum.h"
#include "ctrlib/Utilities.h"
#include "ctrlib/RCRadio3Ch.h"
#include "ctrlib/HsvToRgb.h"
#include "ctrlib/canifier.h"
#include "ctrlib/ctre.h"
#include "ctrlib/PigeonImu.h"
#include "Framework/ServoHoldHeadingWithImu.h"

class Hardware{
public:

	/* Motors and drive-train*/
	static CTRE::MotorControl::CANTalon* leftFront;
	static CTRE::MotorControl::CANTalon* leftRear;
	static CTRE::MotorControl::CANTalon* rightFront;
	static CTRE::MotorControl::CANTalon* rightRear;
	static CTRE::Drive::Mecanum* drivetrain;

	/* Various Controllers */
	static frc::Joystick* gamepad;
	static CTRE::CANifier* canifier;
	static CTRE::RCRadio3Ch* Futuba3Ch;

	//Other stuff used in servos...
	static CTRE::PigeonImu* pigeon;
	static CTRE::Motion::ServoParameters* HoldHeadingParameters;
	static CTRE::Motion::ServoHoldHeadingWithImu* HoldHeadingServo;
};


