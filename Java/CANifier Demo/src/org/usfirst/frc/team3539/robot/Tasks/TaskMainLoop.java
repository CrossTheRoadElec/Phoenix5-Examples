package org.usfirst.frc.team3539.robot.Tasks;

import com.ctre.phoenix.ILoopable;
import org.usfirst.frc.team3539.robot.Platform.Schedulers;
import org.usfirst.frc.team3539.robot.Platform.Hardware;
import org.usfirst.frc.team3539.robot.Platform.Tasks;

public class TaskMainLoop implements ILoopable {
	/* ILoopable */
	public void onStart() {
		/* Default to LED strip animation */
		Schedulers.PeriodicTasks.start(Tasks.taskAnimateLEDStrip);
		Schedulers.PeriodicTasks.stop(Tasks.taskDirectControlArm);
		Schedulers.PeriodicTasks.stop(Tasks.taskLIDAR_ControlLEDStrip);
	}

	public void onStop() {

	}

	public boolean isDone() {
		return false;
	}

	public void onLoop() {
		boolean gamepadPresent = false;
		/*
		 * Don't have the ability to check if game-pad is connected, manually
		 * enable
		 */
		if (gamepadPresent) {
			Schedulers.PeriodicTasks.start(Tasks.taskPWMmotorController);
		} else {
			Schedulers.PeriodicTasks.stop(Tasks.taskPWMmotorController);
		}

		if (Hardware.gamepad.getRawButton(6)) {
			/* Roll through color wheel */
			Schedulers.PeriodicTasks.start(Tasks.taskAnimateLEDStrip);
			Schedulers.PeriodicTasks.stop(Tasks.taskDirectControlArm);
			Schedulers.PeriodicTasks.stop(Tasks.taskLIDAR_ControlLEDStrip);
		} else if (Hardware.gamepad.getRawButton(5)) {
			/* Let user control LED with sticks */
			Schedulers.PeriodicTasks.stop(Tasks.taskAnimateLEDStrip);
			Schedulers.PeriodicTasks.start(Tasks.taskDirectControlArm);
			Schedulers.PeriodicTasks.stop(Tasks.taskLIDAR_ControlLEDStrip);

			Schedulers.PeriodicTasks.start(Tasks.taskMeasurePulseSensors);
		} else if (Hardware.gamepad.getRawButton(7)) {
			/* LED's controlled with the use of LIDAR sensor */
			Schedulers.PeriodicTasks.stop(Tasks.taskAnimateLEDStrip);
			Schedulers.PeriodicTasks.stop(Tasks.taskDirectControlArm);
			Schedulers.PeriodicTasks.start(Tasks.taskLIDAR_ControlLEDStrip);

			Schedulers.PeriodicTasks.start(Tasks.taskMeasurePulseSensors);
		}
	}
}
