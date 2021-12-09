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
#include "frc/TimedRobot.h"
#include "frc/Joystick.h"
#include "frc/drive/MecanumDrive.h"
#include "ctre/Phoenix.h"
#include <iostream>
#include "PhysicsSim.h"

using namespace frc;

class MecanumDefaultCode: public TimedRobot {
	/* Phoenix Talons */
	WPI_TalonSRX lf{0}; /*left front */
	WPI_TalonSRX lr{1};	/*left rear */
	WPI_TalonSRX rf{2}; /*right front */
	WPI_TalonSRX rr{3}; /*right rear */
	WPI_PigeonIMU gyro{0}; /*pigeon IMU */
public:
	void SimulationInit() {
		PhysicsSim::GetInstance().AddTalonSRX(lf, 0.75, 4000);
		PhysicsSim::GetInstance().AddTalonSRX(lr, 0.75, 4000);
		PhysicsSim::GetInstance().AddTalonSRX(rf, 0.75, 4000);
		PhysicsSim::GetInstance().AddTalonSRX(rr, 0.75, 4000);
	}
	void SimulationPeriodic() {
		PhysicsSim::GetInstance().Run();
	}

	void RobotInit(){
		/* Factory Default all hardware to prevent unexpected behaviour */
		lf.ConfigFactoryDefault();
		lr.ConfigFactoryDefault();
		rf.ConfigFactoryDefault();
		rr.ConfigFactoryDefault();
	}
	MecanumDrive m_robotDrive{lf, lr, rf, rr};	// RobotDrive object using PWM 1-4 for drive motors
	Joystick m_driveStick{0};	// Joystick object on USB port 1 (mecanum drive)public:
	/**
	 * Constructor for this "MecanumDefaultCode" Class.
	 */
	MecanumDefaultCode(void) {
		/* Set every Talon to reset the motor safety timeout. */
		lf.Set(0);
		lr.Set(0);
		rf.Set(0);
		rr.Set(0);

		//Invert Right Side
		rf.SetInverted(true);
		rr.SetInverted(true);

		// Create a RobotDrive object using PWMS 1, 2, 3, and 4
		m_robotDrive.SetExpiration(units::time::millisecond_t{500});
		m_robotDrive.SetSafetyEnabled(false);
	}
	void TeleopInit() {
		gyro.Reset();
	}
	/** @return 10% deadband */
	double Db(double axisVal) {
		if (axisVal < -0.10)
			return axisVal;
		if (axisVal > 0.10)
			return axisVal;
		return 0;
	}
	int loopCount = 0;
	/**
	 * Gets called once for each new packet from the DS.
	 */
	void TeleopPeriodic(void) {
		float angle = gyro.GetAngle();
		//std::cout << "Angle : " << angle << std::endl;
		m_robotDrive.DriveCartesian(			Db(m_driveStick.GetX()),
												Db(m_driveStick.GetY()),
												Db(m_driveStick.GetZ()),
												angle);
		if (loopCount++ >= 10) {
			loopCount = 0;
			std::cout << "LF: " << lf.GetMotorOutputPercent() << ", LR: " << lr.GetMotorOutputPercent() <<
				", RF: " << rf.GetMotorOutputPercent() << ", RR: " << rr.GetMotorOutputPercent() << std::endl;
		}
		/* my right side motors need to drive negative to move robot forward */

		/* on button 5, reset gyro angle to zero */
		if (m_driveStick.GetRawButton(5))
			gyro.Reset();
	}
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<MecanumDefaultCode>(); }
#endif
