using System;

/// <summary>
/// Converts hue/saturation/value to RGB.</summary>
/// <remarks>
/// Original posted code found at 
/// http://stackoverflow.com/questions/1335426/is-there-a-built-in-c-net-system-api-for-hsv-to-rgb
/// </remarks>
class HsvToRgb
{
    /// <summary>
    /// Convert hue/saturation/value into RGB values.
    /// </summary>
    /// <param name="hDegrees"> Hue in degrees.</param>
    /// <param name="S">Saturation with range 0 to 1.</param>
    /// <param name="V">Value with range 0 to 1.</param>
    /// <param name="r">Calculated red component.</param>
    /// <param name="g">Calculated green component.</param>
    /// <param name="b">Calculated blue component.</param>
    public static void Convert(double hDegrees, double S, double V, out uint r, out uint g, out uint b)
    {
        double R, G, B;
        double H = hDegrees;

        while (H < 0) { H += 360; };
        while (H >= 360) { H -= 360; };

        if (V <= 0)
        {
            R = G = B = 0;
        }
        else if (S <= 0)
        {
            R = G = B = V;
        }
        else
        {
            double hf = H / 60.0;
            int i = (int)System.Math.Floor(hf);
            double f = hf - i;
            double pv = V * (1 - S);
            double qv = V * (1 - S * f);
            double tv = V * (1 - S * (1 - f));
            switch (i)
            {

                // Red is the dominant color

                case 0:
                    R = V;
                    G = tv;
                    B = pv;
                    break;

                // Green is the dominant color

                case 1:
                    R = qv;
                    G = V;
                    B = pv;
                    break;
                case 2:
                    R = pv;
                    G = V;
                    B = tv;
                    break;

                // Blue is the dominant color

                case 3:
                    R = pv;
                    G = qv;
                    B = V;
                    break;
                case 4:
                    R = tv;
                    G = pv;
                    B = V;
                    break;

                // Red is the dominant color

                case 5:
                    R = V;
                    G = pv;
                    B = qv;
                    break;

                // Just in case we overshoot on our math by a little, we put these here. Since its a switch it won't slow us down at all to put these here.

                case 6:
                    R = V;
                    G = tv;
                    B = pv;
                    break;
                case -1:
                    R = V;
                    G = pv;
                    B = qv;
                    break;

                // The color is not defined, we should throw an error.

                default:
                    //LFATAL("i Value error in Pixel conversion, Value is %d", i);
                    R = G = B = V; // Just pretend its black/white
                    break;
            }
        }
        r = Clamp((int)(R * 255.0));
        g = Clamp((int)(G * 255.0));
        b = Clamp((int)(B * 255.0));
    }

    /// <summary>
    /// Clamp a value to 0-255
    /// </summary>
    private static uint Clamp(int i)
    {
        if (i < 0) return 0;
        if (i > 255) return 255;
        return (uint)i;
    }
}
