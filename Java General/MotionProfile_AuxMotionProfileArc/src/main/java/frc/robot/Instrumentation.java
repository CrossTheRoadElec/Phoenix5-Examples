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
 */
package frc.robot;

import com.ctre.phoenix.motion.*;

public class Instrumentation {

	static double timeout = 0;
	static int count = 0;

	private static final String[] _table = {" Dis ", " En  ", "Hold "};

	public static void OnUnderrun() {
		System.out.format("%s\n", "UNDERRUN");
	}

	public static void OnNoProgress() {
		System.out.format("%s\n", "NOPROGRESS");
	}

	static private String StrOutputEnable(SetValueMotionProfile sv) {
		/* convert sv to string equiv */
		if (sv == null)
			return "null";
		if (sv.value > 3)
			return "Inval";
		return _table[sv.value];
	}

	public static void process(MotionProfileStatus status, double pos,
			double vel, double heading) {
		double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

		if ((now - timeout) > 0.2) {
			timeout = now;
			/* fire a loop every 200ms */

			if (--count <= 0) {
				count = 8;
				/* every 8 loops, print our columns */

				System.out.format("%-9s\t", "outEn");
				System.out.format("%-9s\t", "topCnt");
				System.out.format("%-9s\t", "topRem");
				System.out.format("%-9s\t", "btmCnt");
				System.out.format("%-9s\t", "IsValid");
				System.out.format("%-9s\t", "HasUnder");
				System.out.format("%-9s\t", "IsUnder");
				System.out.format("%-9s\t", "IsLast");
				System.out.format("%-9s\t", "targPos");
				System.out.format("%-9s\t", "targVel");
				System.out.format("%-9s\t", "SlotSel0");
				System.out.format("%-9s\t", "timeDurMs");

				System.out.format("\n");
			}
			/* every loop, print our values */
			System.out.format("%-9s\t", StrOutputEnable(status.outputEnable));
			System.out.format("%-9s\t", status.topBufferCnt);
			System.out.format("%-9s\t", status.topBufferRem);
			System.out.format("%-9s\t", status.btmBufferCnt);
			System.out.format("%-9s\t", (status.activePointValid ? "1" : ""));
			System.out.format("%-9s\t", (status.hasUnderrun ? "1" : ""));
			System.out.format("%-9s\t", (status.isUnderrun ? "1" : ""));
			System.out.format("%-9s\t", (status.isLast ? "1" : ""));
			System.out.format("%-9s\t", pos);
			System.out.format("%-9s\t", vel);
			System.out.format("%-9s\t", status.profileSlotSelect);
			System.out.format("%-9s\t", status.timeDurMs);

			System.out.format("\n");
		}
	}
}