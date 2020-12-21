#pragma once
/**
 * Since this example focuses on Motion Control, lets print everything related to MP in a clean 
 * format.  Expect to see something like......
 * 
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * outputEnable    topBufferRem    topBufferCnt    btmBufferCnt    IsValid     HasUnderrun      IsUnderrun          IsLast         VelOnly         targPos         targVel
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * outputEnable    topBufferRem    topBufferCnt    btmBufferCnt    IsValid     HasUnderrun      IsUnderrun          IsLast         VelOnly         targPos         targVel
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * 
 * ...where the columns are reprinted occasionally so you know whats up.
 * 
 * 
 * 
 */
#include <iomanip> 	// using setw() for printing
#include <iostream> // cout

#include "frc/WPILib.h"
#include "ctre/Phoenix.h"

using namespace frc;

class Instrumentation {
public:
	static void OnNoProgress() {
		std::cout << "NOPROGRESS\n";
	}
	static void OnUnderrun() {
		std::cout << "UNDERRUN\n";
	}
	static const char * StrOutputEnable(unsigned int value) {
		static const char table[][6] = { " Dis ", " En  ", "Hold ", "Inval" };
		if (value > 3)
			value = 3;
		return table[value];
	}
	/**
	 * Prints and/or logging to watch the MP signals
	 */
	static void Process(MotionProfileStatus & status, double pos, double vel,
			double heading) {
		static double timeout = 0;
		static int count = 0;

		const char delim[] = "\t";
		const char endline[] = "\n";

		double now = GetTime();

		if ((now - timeout) > 0.2) {
			timeout = now;
			/* fire a loop every 200ms */

			if (--count <= 0) {
				count = 8;
				/* every 8 loops, print our columns */
				std::cout
							<< "       outEn" << delim
							<< "topBufferCnt" << delim
							<< "topBufferRem" << delim
							<< "btmBufferCnt" << delim
							<< "     IsValid" << delim
							<< " HasUnderrun" << delim
							<< "  IsUnderrun" << delim
							<< "      IsLast" << delim
							<< "     targPos" << delim
							<< "     targVel" << delim
							<< "    SlotSel0" << delim
							<< "   timeDurMs" << delim

							<< endline;
			}
			/* every loop, print our values */
			std::cout	<< std::setw(12)<< StrOutputEnable(status.outputEnable) << delim
						<< std::setw(12)<< status.topBufferCnt << delim
						<< std::setw(12)<< status.topBufferRem << delim
						<< std::setw(12)<< status.btmBufferCnt << delim
						<< std::setw(12)<< (status.activePointValid ? "1" : " ") << delim
						<< std::setw(12)<< (status.hasUnderrun ? "1" : " ") << delim
						<< std::setw(12)<< (status.isUnderrun ? "1" : " ") << delim
						<< std::setw(12)<< (status.isLast ? "1" : " ") << delim
						<< std::setw(12)<< pos << delim
						<< std::setw(12)<< vel << delim
						<< std::setw(12)<< status.profileSlotSelect0 << delim
						<< std::setw(12)<< status.timeDurMs << delim

						<< endline;
		}
	}
};

