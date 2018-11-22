package org.usfirst.frc.team3539.robot.Framework;

public class LinearInterpolation {
	public static float calculate(float x, float x1, float y1, float x2,
			float y2) {
		float m = (y2 - y1) / (x2 - x1);

		float retval = m * (x - x1) + y1;
		return retval;
	}
}
