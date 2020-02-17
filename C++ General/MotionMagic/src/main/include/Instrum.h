#pragma once

#include "frc/smartdashboard/SmartDashboard.h"

#include "ctre/Phoenix.h"
#include <string.h>
#include <iostream>
#include <sstream>
/**
 * Routines for printing to console (FRC Message log).
 */
class Instrum {
    /* Tracking variables for instrumentation */
private: 
    static int _loops;
	static int _timesInMotionMagic;

public:
    static void Process(TalonSRX *tal, std::stringstream *sb) {
		/* Smart dash plots */
		frc::SmartDashboard::PutNumber("SensorVel", tal->GetSelectedSensorVelocity(0));
		frc::SmartDashboard::PutNumber("SensorPos", tal->GetSelectedSensorPosition(0));
		frc::SmartDashboard::PutNumber("MotorOutputPercent", tal->GetMotorOutputPercent());
		frc::SmartDashboard::PutNumber("ClosedLoopError", tal->GetClosedLoopError(0));
		
		/* Check if Talon SRX is performing Motion Magic */
		if (tal->GetControlMode() == ControlMode::MotionMagic) {
			++_timesInMotionMagic;
		} else {
			_timesInMotionMagic = 0;
		}

		if (_timesInMotionMagic > 10) {
			/* Print the Active Trajectory Point Motion Magic is servoing towards */
			frc::SmartDashboard::PutNumber("ClosedLoopTarget", tal->GetClosedLoopTarget(0));
    		frc::SmartDashboard::PutNumber("ActTrajVelocity", tal->GetActiveTrajectoryVelocity());
    		frc::SmartDashboard::PutNumber("ActTrajPosition", tal->GetActiveTrajectoryPosition());
		}

		/* Periodically print to console */
		if (++_loops >= 20) {
			_loops = 0;
			std::cout << sb->str() << std::endl;
		}
	}
};

int Instrum::_loops = 0;
int Instrum::_timesInMotionMagic = 0;
