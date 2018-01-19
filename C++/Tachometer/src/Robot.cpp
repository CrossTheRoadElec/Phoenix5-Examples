/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <IterativeRobot.h>
#include <WPILib.h>
#include "ctre/Phoenix.h"

class Robot: public frc::IterativeRobot {
public:

	TalonSRX * _magTalon;
	TalonSRX * _tachTalon;
	Joystick * _joy;

	void RobotInit() {
		_magTalon = new TalonSRX(4);
		_tachTalon = new TalonSRX(5);
		_joy = new Joystick(0);
	}

	void TeleopInit() {
		//Configure talon to read magencoder values
		_magTalon->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

		//Configure talon to read tachometer values
		_tachTalon->ConfigSelectedFeedbackSensor(
				FeedbackDevice::Tachometer, 0, 10);
		//Edges per cycle = 2 (WHTIE black WHITE black)
		_tachTalon->ConfigSetParameter((ParamEnum) 430, 2, 0, 0, 10);
		//Cycles per rev = 1
		_tachTalon->ConfigSetParameter((ParamEnum) 431, 1, 0, 0, 10);
	}

	void TeleopPeriodic() {
		_magTalon->Set(ControlMode::PercentOutput, _joy->GetY());

		//MagRPM = u/100ms * 600(100ms/m) / 4096(u/rev) = rev/m
		double magRPM = _magTalon->GetSelectedSensorVelocity(0) * 600 / 4096;
		//TachRPM = u/100ms * 600(100ms/m) / 1024(u/rev) = rev/m
		double tachRPM = _tachTalon->GetSelectedSensorVelocity(0) * 600 / 1024;

		//Write to DS
		std::cout << "Mag encoder speed is: " << magRPM << "\t"
				<< "Tach speed is: " << tachRPM << std::endl;
	}

	void TestPeriodic() {
	}

private:
};

START_ROBOT_CLASS(Robot)
