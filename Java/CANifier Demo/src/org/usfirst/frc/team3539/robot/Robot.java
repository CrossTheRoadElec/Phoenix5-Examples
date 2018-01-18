/**
 * IMPORTANT: This example requires the version of the SDK from the
 * Installer version 5.0.3.2 or higher.  There were several changes
 * and additions to multiple files in the SDK, and these are required
 * for this example
 * 
 * Anything under the "Framework" folder are tentative changes later to be into the Phoenix Framework.
 * 
 * Anything under "Tasks" are typical examples tasks done on a robot platform.
 */

package org.usfirst.frc.team3539.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import com.ctre.phoenix.ILoopable;
import org.usfirst.frc.team3539.robot.Platform.*;

public class Robot extends IterativeRobot {
	@Override
	public void robotInit() {
	}

	@Override
	public void teleopInit() {
		/* Add each task to the concurrent scheduler */
		for (ILoopable loop : Tasks.FullList) {
			Schedulers.PeriodicTasks.add(loop);
		}
	}

	@Override
	public void teleopPeriodic() {
		/** Run forever */

		/* Process the concurrent scheduler which will process our tasks */
		Schedulers.PeriodicTasks.process();
	}
}
