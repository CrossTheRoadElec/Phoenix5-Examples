#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <thread>
#include <unistd.h>
#include "ctre/Phoenix.h"

class PlotThread
{
private:
    std::thread *_thread;
public:
    PlotThread(TalonSRX *talon)
    {
        _thread = new std::thread(Run, talon);
    }

    static void Run(TalonSRX *_talon) 
    {
        /**
		 * Speed up network tables, this is a test project so eat up all of the network
		 * possible for the purpose of this test.
		 */

		while (true) {
			/* Yield for a Ms or so - this is not meant to be accurate */
			usleep(1000);

			/* Grab the latest signal update from our 1ms frame update */
			double sen_pos = _talon->GetSelectedSensorPosition(0);
			double sen_vel = _talon->GetSelectedSensorVelocity(0);
			double trgt_pos = _talon->GetActiveTrajectoryPosition(0);
			double trgt_vel = _talon->GetActiveTrajectoryVelocity(0);
			double trgt_arbF = _talon->GetActiveTrajectoryArbFeedFwd(0);
			frc::SmartDashboard::PutNumber("sen_pos", sen_pos);
			frc::SmartDashboard::PutNumber("sen_vel", sen_vel);
			frc::SmartDashboard::PutNumber("trgt_pos", trgt_pos);
			frc::SmartDashboard::PutNumber("trgt_vel", trgt_vel);
			frc::SmartDashboard::PutNumber("trgt_arbF", trgt_arbF);
		}
    }
};
