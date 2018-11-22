package org.usfirst.frc.team3539.robot.Tasks;

import com.ctre.phoenix.ILoopable;
import org.usfirst.frc.team3539.robot.Framework.LinearInterpolation;
import org.usfirst.frc.team3539.robot.Platform.Constants;
import org.usfirst.frc.team3539.robot.Platform.Hardware;

public class TaskPWMmotorController implements ILoopable {
	float _percentOut;
	boolean _running; /* Assists TaskMainLoop with tracking, used within init */

	/* ILoopable */
	public void onStart() {
		/* If we are already running, nothing to do */
		if (_running) {
			return;
		}

		/* Start transmitting neutral */
		_percentOut = 0;
		Hardware.canifier.setPWMOutput(Constants.kMotorControllerCh.value, 0);
		Hardware.canifier.enablePWMOutput(Constants.kMotorControllerCh.value,
				true);

		/* Task is now running */
		_running = true;
	}

	public void onStop() {
		/* Stop transmitting PWM */
		Hardware.canifier.enablePWMOutput(Constants.kMotorControllerCh.value,
				false);

		/* Task has stopped */
		_running = false;
	}

	public boolean isDone() {
		return false;
	}

	public void onLoop() {
		/* Grab three axis and direct control the PWM MotorController */
		float axis = (float) Hardware.gamepad
				.getRawAxis(Constants.GamePadAxis_y);
		/* Scale to typical PWM widths */
		float pulseUs = LinearInterpolation.calculate(axis, -1, 1000f, +1,
				2000f); /* [-1,+1] => [1000,2000]us */
		/* Scale to period */
		float periodUs = 4200; /*
								 * Hard-coded for now, this will be settable in
								 * future firmware update.
								 */
		_percentOut = pulseUs / periodUs;
		/* Set PWM Motor Controller */
		Hardware.canifier.setPWMOutput(Constants.kMotorControllerCh.value,
				_percentOut);
	}

	public String toString() {
		return "TaskPWMmotorController:";
	}
}
