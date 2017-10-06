//#include "Platform/Hardware.h"
//
///* Motors and drive-train*/
//CTRE::MotorControl::CANTalon* Hardware::leftFront = new CTRE::MotorControl::CANTalon(1);
//CTRE::MotorControl::CANTalon* Hardware::leftRear = new CTRE::MotorControl::CANTalon(2);
//CTRE::MotorControl::CANTalon* Hardware::rightFront = new CTRE::MotorControl::CANTalon(3);
//CTRE::MotorControl::CANTalon* Hardware::rightRear = new CTRE::MotorControl::CANTalon(4);
//CTRE::Drive::Mecanum* Hardware::drivetrain = new CTRE::Drive::Mecanum(leftFront, leftRear, rightFront,
//			rightRear);
//
///* Various Controllers */
//frc::Joystick* Hardware::gamepad = new frc::Joystick(0);
////
//CTRE::CANifier* Hardware::canifier = new CTRE::CANifier(0);
//CTRE::RCRadio3Ch* Hardware::Futuba3Ch = new CTRE::RCRadio3Ch(canifier);
//
///* Other stuff used in servos... */
//CTRE::PigeonImu* Hardware::pigeon = new CTRE::PigeonImu(1);
//
//CTRE::Motion::ServoParameters* Hardware::HoldHeadingParameters = new CTRE::Motion::ServoParameters();
//
//CTRE::Motion::ServoHoldHeadingWithImu* Hardware::HoldHeadingServo =
//		new CTRE::Motion::ServoHoldHeadingWithImu(pigeon,
//					drivetrain,
//					CTRE::Drive::Styles::Basic::PercentOutputBasic,
//					HoldHeadingParameters, 0, 0.2);
