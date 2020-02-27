/**
 * TaskDirectControlLEDStrip Class
 * 
 * ILoopable Task for manually controlling/selecting HSV value through gamepad joysticks.
 * Updates values for taskHSV_ControlLedStrip, interacting with the TaskHSV.java when in 
 * scheduler.
 * 
 * Controls:
 * Right Joystick (Both Y and X): Select HSV Value by rotating stick,
 * Position of stick should represent position on HSV Color Circle.
 */
package frc.robot.Tasks;

import frc.robot.Platform.*;
import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.Util;

public class TaskDirectControlLEDStrip implements ILoopable {
    
    /* ILoopable */
	public void onStart() {	}

	public void onStop() {	}

	public boolean isDone() { return false; }

	public void onLoop() {
		/* Get an x and y pair */
		float x = (float) Hardware.gamepad.getRawAxis(Constants.GamePadAxis_x);
		float y = (float) Hardware.gamepad.getRawAxis(Constants.GamePadAxis_y);

		/* Calculate theta in degrees */
		float theta = (float) Math.atan2(x, y) * 180f / (float) Math.PI;
		/*
		 * Take the magnitude and cap it at '1'. This will be our saturation
		 * (how far away from white we want to be)
		 */
		float saturation = (float) Math.sqrt(x * x + y * y);
		saturation = (float) Util.cap(saturation, 1);
		/* Pick a value of '1', how far away from black we want to be. */
		Tasks.taskHSV_ControlLedStrip.Hue = theta;
		Tasks.taskHSV_ControlLedStrip.Saturation = saturation;
		Tasks.taskHSV_ControlLedStrip.Value = 1; /* scale down for brightness */
	}
                                                                                          
	public String toString() {
		return "ControlLEDStrip:";
	}
}
