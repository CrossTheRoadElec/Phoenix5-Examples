/**
 * TaskMainLoop Class
 * 
 * ILoopable Task for managing the multiple tasks in the scheduler
 * The main loop tasks starts and stops tasks according to gamepad state and
 * requested operation mode. 
 * 
 * Controls:
 * Button 6: Simply allow CANifier to control LED by cycling HSV wheel
 * Button 5: Use Joysticks to Directly control HSV values for LED Strip
 * 	+ Measure all PWM channels on CANifier if enabled
 * Button 7: Use LIDAR to control control HSV values for LED Strip 
 * 	+ Measure all PWM channels on CANifier if enabled
 * Set gamepadPresent to True to enable PWM Motor Control with Gamepad
 */
package frc.robot.Tasks;

import frc.robot.Platform.Schedulers;
import frc.robot.Platform.Hardware;
import frc.robot.Platform.Tasks;
import com.ctre.phoenix.ILoopable;

public class TaskMainLoop implements ILoopable {

	/* ILoopable */
	public void onStart() {
		/* Default to LED strip animation */
		Schedulers.PeriodicTasks.start(Tasks.taskAnimateLEDStrip);
		Schedulers.PeriodicTasks.stop(Tasks.taskDirectControlArm);
		Schedulers.PeriodicTasks.stop(Tasks.taskLIDAR_ControlLEDStrip);
	}

	public void onStop() { }

	public boolean isDone() { return false; }

	public void onLoop() {
        /**
		 * Don't have the ability to check if game-pad is connected...
         * manually enable byt setting gamepadPresent to true;
		 */
        boolean gamepadPresent = false;
		if (gamepadPresent) {
			Schedulers.PeriodicTasks.start(Tasks.taskPWMmotorController);
		} else {
			Schedulers.PeriodicTasks.stop(Tasks.taskPWMmotorController);
		}

		if (Hardware.gamepad.getRawButton(6)) {         // Right-Bumper-Button
			/* Roll through color wheel */
			Schedulers.PeriodicTasks.start(Tasks.taskAnimateLEDStrip);
			Schedulers.PeriodicTasks.stop(Tasks.taskDirectControlArm);
			Schedulers.PeriodicTasks.stop(Tasks.taskLIDAR_ControlLEDStrip);
		} else if (Hardware.gamepad.getRawButton(5)) {  // Left-Bumper-Button
			/* Let user control LED with sticks */
			Schedulers.PeriodicTasks.stop(Tasks.taskAnimateLEDStrip);
			Schedulers.PeriodicTasks.start(Tasks.taskDirectControlArm);
			Schedulers.PeriodicTasks.stop(Tasks.taskLIDAR_ControlLEDStrip);
            /* Measure all PWM Inputs */
			Schedulers.PeriodicTasks.start(Tasks.taskMeasurePulseSensors);
		} else if (Hardware.gamepad.getRawButton(7)) {  // Left-Trigger-Button
			/* LED's controlled with the use of LIDAR sensor */
			Schedulers.PeriodicTasks.stop(Tasks.taskAnimateLEDStrip);
			Schedulers.PeriodicTasks.stop(Tasks.taskDirectControlArm);
			Schedulers.PeriodicTasks.start(Tasks.taskLIDAR_ControlLEDStrip);
            /* Measure all PWM Inputs */
			Schedulers.PeriodicTasks.start(Tasks.taskMeasurePulseSensors);
		}
	}
}
