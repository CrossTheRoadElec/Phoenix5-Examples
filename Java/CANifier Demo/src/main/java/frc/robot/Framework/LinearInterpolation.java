/**
 * LinearInterpolation Class
 * 
 * Framework Class, meaning it will be added to Phoenix API in the future.
 * 
 * Class containing function to convert x value provided to fit within the 
 * two provided points, regardless of if it has to scale up or down to be in range.
 */
package frc.robot.Framework;

public class LinearInterpolation {
	public static float calculate(float x, float x1, float y1, float x2,
			float y2) {
		float m = (y2 - y1) / (x2 - x1);

		float retval = m * (x - x1) + y1;
		return retval;
	}
}
