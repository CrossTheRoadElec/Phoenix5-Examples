package org.usfirst.frc.team3539.robot.Platform;

import org.usfirst.frc.team3539.robot.Tasks.TaskAnimateLEDStrip;
import org.usfirst.frc.team3539.robot.Tasks.TaskDirectControlLEDStrip;
import org.usfirst.frc.team3539.robot.Tasks.TaskHSV;
import org.usfirst.frc.team3539.robot.Tasks.TaskLIDAR_ControlLEDStrip;
import org.usfirst.frc.team3539.robot.Tasks.TaskMeasurePulseSensors;
import org.usfirst.frc.team3539.robot.Tasks.TaskPWMmotorController;
import org.usfirst.frc.team3539.robot.Tasks.TaskMainLoop;
import com.ctre.phoenix.ILoopable;

public class Tasks {
	/*
	 * Subsystem tasks. A task could be all the functionality of a subsystem
	 * like a robot arm, or could be an individual action to a subsystem, such
	 * as up reading a game-pad and applying it to the drive-train. Use 'public
	 * static' because these are single objects.
	 */
	public static TaskAnimateLEDStrip taskAnimateLEDStrip = new TaskAnimateLEDStrip();
	public static TaskDirectControlLEDStrip taskDirectControlArm = new TaskDirectControlLEDStrip();
	public static TaskPWMmotorController taskPWMmotorController = new TaskPWMmotorController();
	public static TaskMeasurePulseSensors taskMeasurePulseSensors = new TaskMeasurePulseSensors();
	public static TaskLIDAR_ControlLEDStrip taskLIDAR_ControlLEDStrip = new TaskLIDAR_ControlLEDStrip();
	public static TaskHSV taskHSV_ControlLedStrip = new TaskHSV();

	public static TaskMainLoop taskMainLoop = new TaskMainLoop();

	/*
	 * Insert all Tasks below in the Full List so they get auto inserted, see
	 * Robot.java to see how this works.
	 */
	public static ILoopable[] FullList = {taskAnimateLEDStrip,
			taskDirectControlArm, taskPWMmotorController,
			taskMeasurePulseSensors, taskLIDAR_ControlLEDStrip,
			taskHSV_ControlLedStrip, taskMainLoop,};
}
