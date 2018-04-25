/**
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
 * API Libraries ONLY when in use with Cross The Road Electronics hardware products.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Converts hue, saturation, and value into RGB
 * 
 * Original posted code found at
 * http://stackoverflow.com/questions/1335426/is-there-a-built-in-c-net-system-api-for-hsv-to-rgb
 */

using System;
class HsvToRgb
{
    /**
     * Convert hue/saturation/and value into RGB values
     * 
     * @param   hDegrees    Hue in degrees
     * @param   S           Saturation with range of 0 to 1
     * @param   V           Value with range of 0 to 1
     * @param   r           Calculated Red value of RGB
     * @param   g           Calculated Green value of RGB
     * @param   b           Calculated Blue value of RGB
     */
    public static void Convert(double hDegrees, double S, double V, out uint r, out uint g, out uint b)
    {
        double R, G, B;
        double H = hDegrees;

        if (H < 0) { H += 360; };
        if (H >= 360) { H -= 360; };

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
                /* Red is dominant color */
                case 0:
                    R = V;
                    G = tv;
                    B = pv;
                    break;
                /* Green is dominant color */
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
                /* Blue is the dominant color */
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
                /* Red is the dominant color */
                case 5:
                    R = V;
                    G = pv;
                    B = qv;
                    break;
                /* Just in case we overshoot on our math by a little, we put these here. Since its a switch it won't slow us down at all to put these here */
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
                /* The color is not defined, we should throw an error */
                default:
                    /* Just pretend its black/white */
                    R = G = B = V;
                    break;
            }
        }
        r = Clamp((int)(R * 255.0));
        g = Clamp((int)(G * 255.0));
        b = Clamp((int)(B * 255.0));
    }

    /** Clamp a value to 0 - 255 */
    private static uint Clamp(int i)
    {
        if (i < 0) return 0;
        if (i > 255) return 255;
        return (uint)i;
    }
}
