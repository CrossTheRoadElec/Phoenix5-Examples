package org.usfirst.frc.team217.robot.Platform;

import com.ctre.phoenix.ILoopable;
import org.usfirst.frc.team217.Tasks.TaskTankDriveAndPositionClosedLoop;

public class Tasks {
	public static TaskTankDriveAndPositionClosedLoop taskDrive = new TaskTankDriveAndPositionClosedLoop();
	
	public static ILoopable[] FullList = {
			taskDrive,
	};
}
