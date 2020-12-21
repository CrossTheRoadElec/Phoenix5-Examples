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
 * This C++ FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon SRX.  The CANTalon class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon SRX.
 * 
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 * 
 * This application is an IterativeRobot project to demonstrate a minimal implementation not requiring the command 
 * framework, however these code excerpts could be moved into a command-based project.
 * 
 * The project also includes Instrumentation.h which simply has debug printfs, and a MotionProfile.h which is generated
 * in @link https://docs.google.com/spreadsheets/d/1PgT10EeQiR92LNXEOEe3VGn737P7WDP4t0CQxQgC8k0/edit#gid=1813770630&vpid=A1
 * or use the Motion Profile Generator.xlsx file in the project folder.
 * 
 * Logitech Gamepad mapping, use left y axis to drive Talon normally.  
 * Press and hold top-left-shoulder-button5 to put Talon into motion profile control mode.
 * This will start sending Motion Profile to Talon while Talon is neutral. 
 * 
 * While holding top-left-shoulder-button5, tap top-right-shoulder-button6.
 * This will signal Talon to fire MP.  When MP is done, Talon will "hold" the last setpoint position
 * and wait for another button6 press to fire again.
 * 
 * Release button5 to allow OpenVoltage control with left y axis.
 */
#include "frc/WPILib.h"
#include "Instrumentation.h"
#include "MotionProfileExample.h"
#include "ctre/Phoenix.h"
#include "Constants.h"
#include "PhysicsSim.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	/** The Talon we want to motion profile. */
	WPI_TalonSRX _talon;
	WPI_VictorSPX _vic;

	/** some example logic on how one can manage an MP */
	MotionProfileExample _example;

	/** joystick for testing */
	Joystick _joy;

	/** cache last buttons so we can detect press events.  In a command-based project you can leverage the on-press event
	 * but for this simple example, lets just do quick compares to prev-btn-states */
	bool _btnsLast[10] = { false, false, false, false, false, false, false, false, false, false };

	Robot() :
			_talon(Constants::kTalonID), _vic(Constants::kVictorFollower), _example(
					_talon), _joy(0) {
	}
	void SimulationInit() {
		PhysicsSim::GetInstance().AddTalonSRX(_talon, 0.75, 2000, true);
		PhysicsSim::GetInstance().AddVictorSPX(_vic);
	}
	void SimulationPeriodic() {
		PhysicsSim::GetInstance().Run();
	}
	void RobotInit(){
	/* Factory Default all hardware to prevent unexpected behaviour */
		_talon.ConfigFactoryDefault();
		_vic.ConfigFactoryDefault();
	}
	/** run once after booting/enter-disable */
	void DisabledInit() {
		_vic.Follow(_talon);
		_talon.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,
				kTimeoutMs);
		_talon.SetSensorPhase(true);
		_talon.ConfigNeutralDeadband(Constants::kNeutralDeadbandPercent * 0.01,
				Constants::kTimeoutMs);

		_talon.Config_kF(0, 0.076, kTimeoutMs);
		_talon.Config_kP(0, 2.000, kTimeoutMs);
		_talon.Config_kI(0, 0.0, kTimeoutMs);
		_talon.Config_kD(0, 20.0, kTimeoutMs);

		_talon.ConfigMotionProfileTrajectoryPeriod(10, Constants::kTimeoutMs); //Our profile uses 10 ms timing
		/* status 10 provides the trajectory target for motion profile AND motion magic */
		_talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic,
				10, Constants::kTimeoutMs);
	}
	/**  function is called periodically during operator control */
	void TeleopPeriodic() {
		/* get buttons */
		bool btns[10];
		for (unsigned int i = 1; i < 10; ++i)
			btns[i] = _joy.GetRawButton(i);

		/* get the left joystick axis on Logitech Gampead */
		double leftYjoystick = -1 * _joy.GetY(); /* multiple by -1 so joystick forward is positive */

		/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
		_example.control();
		_example.PeriodicTask();

		if (btns[5] == false) { /* Check button 5 (top left shoulder on the logitech gamead). */
			/*
			 * If it's not being pressed, just do a simple drive.  This
			 * could be a RobotDrive class or custom drivetrain logic.
			 * The point is we want the switch in and out of MP Control mode.*/

			/* button5 is off so straight drive */
			_talon.Set(ControlMode::PercentOutput, 1.0 * leftYjoystick);

			_example.reset();
		} else {
			/* Button5 is held down so switch to motion profile control mode => This is done in MotionProfileControl.
			 * When we transition from no-press to press,
			 * pass a "true" once to MotionProfileControl.
			 */

			SetValueMotionProfile setOutput = _example.getSetValue();

			_talon.Set(ControlMode::MotionProfile, setOutput);

			/* if btn is pressed and was not pressed last time,
			 * In other words we just detected the on-press event.
			 * This will signal the robot to start a MP */
			if ((btns[6] == true) && (_btnsLast[6] == false)) {
				/* user just tapped button 6 */

				//------------ We could start an MP if MP isn't already running ------------//
				_example.start();
			}
		}

		/* save buttons states for on-press detection */
		for (int i = 1; i < 10; ++i)
			_btnsLast[i] = btns[i];

	}
	/**  function is called periodically during disable */
	void DisabledPeriodic() {
		/* it's generally a good idea to put motor controllers back
		 * into a known state when robot is disabled.  That way when you
		 * enable the robot doesn't just continue doing what it was doing before.
		 * BUT if that's what the application/testing requires than modify this accordingly */
		_talon.Set(ControlMode::PercentOutput, 0);
		/* clear our buffer and put everything into a known state */
		_example.reset();
	}
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
