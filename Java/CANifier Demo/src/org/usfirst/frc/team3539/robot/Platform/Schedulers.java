package org.usfirst.frc.team3539.robot.Platform;

import org.usfirst.frc.team3539.robot.Framework.ConcurrentScheduler;

public class Schedulers {
	/*
	 * the schedulers. Minimally you will likely want one periodic scheduler to
	 * run the normal tasks. Additional schedulers could be
	 * ConsecutiveSchedulers for entire autonomous movements or pre-configured
	 * maneuvers. Use 'public static' because these are single objects.
	 */
	public static ConcurrentScheduler PeriodicTasks = new ConcurrentScheduler();
}
