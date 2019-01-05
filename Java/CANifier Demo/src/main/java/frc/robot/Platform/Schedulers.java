/** 
 * Simple Class containing schedulers used throughout project
 */
package frc.robot.Platform;

import com.ctre.phoenix.schedulers.ConcurrentScheduler;

public class Schedulers {
	/**
	 * The schedulers. Minimally you will likely want one periodic scheduler to
	 * run the normal tasks. Additional schedulers could be
	 * ConsecutiveSchedulers for entire autonomous movements or pre-configured
	 * maneuvers. Use 'public static' because these are single objects.
	 */
	public static ConcurrentScheduler PeriodicTasks = new ConcurrentScheduler();
}
