/**
 * HsvToRgb Class
 * 
 * Framework Class, meaning it will be added to Phoenix API in the future
 * 
 * Class containing function to handle the conversion of HSV values into
 * RGB values for color equivalency. 
 */
package frc.robot.Framework;

public class HsvToRgb {
	/**
	 * Convert hue/saturation/and value into RGB values
	 * 
	 * @param hDegrees  Hue in degrees
	 * @param S         Saturation with range of 0 to 1
	 * @param V         Value with range of 0 to 1
	 */
	static float RGB[] = new float[3];

	public static float[] convert(double hDegrees, double S, double V) {
		double R, G, B;
		double H = hDegrees;

		if (H < 0) {
			H += 360;
		}
		if (H >= 360) {
			H -= 360;
		}

		if (V <= 0) {
			R = G = B = 0;
		} else if (S <= 0) {
			R = G = B = V;
		} else {
			double hf = H / 60.0;
			int i = (int) Math.floor(hf);
			double f = hf - i;
			double pv = V * (1 - S);
			double qv = V * (1 - S * f);
			double tv = V * (1 - S * (1 - f));
			switch (i) {
				/* Red is dominant color */
				case 0 :
					R = V;
					G = tv;
					B = pv;
					break;
				/* Green is dominant color */
				case 1 :
					R = qv;
					G = V;
					B = pv;
					break;
				case 2 :
					R = pv;
					G = V;
					B = tv;
					break;
				/* Blue is the dominant color */
				case 3 :
					R = pv;
					G = qv;
					B = V;
					break;
				case 4 :
					R = tv;
					G = pv;
					B = V;
					break;
				/* Red is the dominant color */
				case 5 :
					R = V;
					G = pv;
					B = qv;
					break;
				/**
				 * Just in case we overshoot on our math by a little, we put
				 * these here. Since its a switch it won't slow us down at all
				 * to put these here
				 */
				case 6 :
					R = V;
					G = tv;
					B = pv;
					break;
				case -1 :
					R = V;
					G = pv;
					B = qv;
					break;
				/* The color is not defined, we should throw an error */
				default :
					/* Just pretend its black/white */
					R = G = B = V;
					break;
			}
		}
		/* Since we can't pass by reference, return an array */
		RGB[0] = (float) R;
		RGB[1] = (float) G;
		RGB[2] = (float) B;

		return RGB;
	}
}
