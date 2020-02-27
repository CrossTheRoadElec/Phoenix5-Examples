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
 * Enable robot and slowly drive forward.
 * [1] If DS reports errors, adjust CAN IDs and firmware update.
 * [2] If motors are spinning incorrectly, first check gamepad.
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 */
#include <iostream>
#include <string>

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	/* ------ [1] Update CAN Device IDs and switch to WPI_VictorSPX where necessary ------*/
	WPI_TalonFX * _rghtFront = new WPI_TalonFX(1);
	WPI_TalonFX * _rghtFollower = new WPI_TalonFX(3);
	WPI_TalonFX * _leftFront = new WPI_TalonFX(2);
	WPI_TalonFX * _leftFollower = new WPI_TalonFX(4);

	DifferentialDrive * _diffDrive = new DifferentialDrive(*_leftFront,
			*_rghtFront);

	Joystick * _joystick = new Joystick(0);

	void TeleopPeriodic() {

		std::stringstream work;

		/* get gamepad stick values */
		double forw = -1 * _joystick->GetRawAxis(1); /* positive is forward */
		double turn = +1 * _joystick->GetRawAxis(2); /* positive is right */

		/* deadband gamepad 10%*/
		if (fabs(forw) < 0.10)
			forw = 0;
		if (fabs(turn) < 0.10)
			turn = 0;

		/* drive robot */
		_diffDrive->ArcadeDrive(forw, turn, false);

		/* -------- [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for RIGHT */
		work << " GF:" << forw << " GT:" << turn;

		/* get sensor values */
		//double leftPos = _leftFront->GetSelectedSensorPosition(0);
		//double rghtPos = _rghtFront->GetSelectedSensorPosition(0);
		double leftVelUnitsPer100ms = _leftFront->GetSelectedSensorVelocity(0);
		double rghtVelUnitsPer100ms = _rghtFront->GetSelectedSensorVelocity(0);

		work << " L:" << leftVelUnitsPer100ms << " R:" << rghtVelUnitsPer100ms;

		/* print to console */
		std::cout << work.str() << std::endl;
	}

	void RobotInit() {
		/* factory default values */
		_rghtFront->ConfigFactoryDefault();
		_rghtFollower->ConfigFactoryDefault();
		_leftFront->ConfigFactoryDefault();
		_leftFollower->ConfigFactoryDefault();

		/* set up followers */
		_rghtFollower->Follow(*_rghtFront);
		_leftFollower->Follow(*_leftFront);

		/* [3] flip values so robot moves forward when stick-forward/LEDs-green */
		_rghtFront->SetInverted(TalonFXInvertType::Clockwise);
		_rghtFollower->SetInverted(TalonFXInvertType::FollowMaster);
		_leftFront->SetInverted(TalonFXInvertType::CounterClockwise);
		_leftFollower->SetInverted(TalonFXInvertType::FollowMaster);

		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
		// _rghtFront->SetSensorPhase(true);
		// _leftFront->SetSensorPhase(true);

		/*
		* WPI drivetrain classes defaultly assume left and right are opposite. call
		* this so we can apply + to both sides when moving forward. DO NOT CHANGE
		*/
		_diffDrive->SetRightSideInverted(false);
	}

private:
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif