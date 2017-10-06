#include "WPILIB.h"

#include "Platform/Hardware.h"
#include "Platform/Schedulers.h"
#include "Platform/Tasks.h"
#include <chrono>
#include <thread>

CTRE::MotorControl::CANTalon* Hardware::leftFront;
CTRE::MotorControl::CANTalon* Hardware::leftRear;
CTRE::MotorControl::CANTalon* Hardware::rightFront;
CTRE::MotorControl::CANTalon* Hardware::rightRear;
CTRE::Drive::Mecanum* Hardware::drivetrain;
frc::Joystick* Hardware::gamepad;
CTRE::CANifier* Hardware::canifier;
CTRE::RCRadio3Ch* Hardware::Futuba3Ch;
CTRE::PigeonImu* Hardware::pigeon;
CTRE::Motion::ServoParameters* Hardware::HoldHeadingParameters;
CTRE::Motion::ServoHoldHeadingWithImu* Hardware::HoldHeadingServo;

class Robot: public frc::IterativeRobot {
public:
	void RobotInit() {
		/* Create objects after initialization due to bug */

		/* Motors and drive-train*/
		Hardware::leftFront = new CTRE::MotorControl::CANTalon(1);
		Hardware::leftRear = new CTRE::MotorControl::CANTalon(2);
		Hardware::rightFront = new CTRE::MotorControl::CANTalon(4);
		Hardware::rightRear = new CTRE::MotorControl::CANTalon(3);
		Hardware::drivetrain = new CTRE::Drive::Mecanum(Hardware::leftFront, Hardware::leftRear, Hardware::rightFront,
				Hardware::rightRear);

		/* Various Controllers */
		Hardware::gamepad = new frc::Joystick(0);
		//
		Hardware::canifier = new CTRE::CANifier(0);
		Hardware::Futuba3Ch = new CTRE::RCRadio3Ch(Hardware::canifier);

		/* Other stuff used in servos... */
		Hardware::pigeon = new CTRE::PigeonImu(Hardware::leftRear);

		Hardware::HoldHeadingParameters = new CTRE::Motion::ServoParameters();

		Hardware::HoldHeadingServo =
				new CTRE::Motion::ServoHoldHeadingWithImu(Hardware::pigeon,
						Hardware::drivetrain,
							CTRE::Drive::Styles::Basic::PercentOutputBasic,
							Hardware::HoldHeadingParameters, 0, 0.2);
	}

	void TeleopInit(){
		/* Invert one side of the robot, may have to be changed by user */
		Hardware::leftFront->SetInverted(true);
		Hardware::leftRear->SetInverted(true);

		/* Insert periodic tasks into a concurrent scheduler */
		Schedulers::PeriodicTasks->Add(Tasks::EnableRobot);
		Schedulers::PeriodicTasks->Add(Tasks::LEDStrip);
		Schedulers::PeriodicTasks->Add(Tasks::TeleopDriveWithRC);
		Schedulers::PeriodicTasks->Add(Tasks::TeleopDriveWithGamepad);
		Schedulers::PeriodicTasks->Add(Tasks::LowBatteryDetect);
	}

	void TeleopPeriodic() {
		/* Process concurrent scheduler, which processes each Periodic task */
		Schedulers::PeriodicTasks->Process();

		/* Process RC Radio Controller to attain values */
		Hardware::Futuba3Ch->Process();
	}
};

START_ROBOT_CLASS(Robot)
