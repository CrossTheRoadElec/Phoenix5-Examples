/**
 * All debug print statements are placed in this module for clarity.
 * VIEW => OUTPUT to see the output window.
 * This module could be modified to use UART, CAN, etc...
 */
using System;
using Microsoft.SPOT;
using CTRE.Phoenix.MotorControl.CAN;

namespace HERO_Motion_Magic_Example
{
    public static class Instrument
    {
        /** create a single stringbuilder to reuse */
        private static System.Text.StringBuilder _sb = new System.Text.StringBuilder();
        /** counts loops to reduce print frequency */
        private static int _instrumLoops1 = 0;
        /** number of inner loops to occasionally print columns */
        private static int _instrumLoops2 = 0;

        public static void PrintConfigError()
        {
            Debug.Print("Initialization failed, will try again...");
        }
        public static void Process(TalonSRX talon)
        {
            /* simple timeout to reduce printed lines */
            if (++_instrumLoops1 > 10)
            {
                _instrumLoops1 = 0;

                /* get closed loop info */
                float pos = talon.GetSelectedSensorPosition(0);
                float spd = talon.GetSelectedSensorVelocity(0);
                int err = talon.GetClosedLoopError(0);

                /* build the string */
                _sb.Clear();

                _sb.Append(pos);
                if (_sb.Length < 16) { _sb.Append(' ', 16 - _sb.Length); }

                _sb.Append(spd);
                if (_sb.Length < 32) { _sb.Append(' ', 32 - _sb.Length); }

                _sb.Append(err);
                if (_sb.Length < 48) { _sb.Append(' ', 48 - _sb.Length); }

                Debug.Print(_sb.ToString()); /* print data row */

                if (++_instrumLoops2 > 8)
                {
                    _instrumLoops2 = 0;

                    _sb.Clear();

                    _sb.Append("Position");
                    if (_sb.Length < 16) { _sb.Append(' ', 16 - _sb.Length); }

                    _sb.Append("Velocity");
                    if (_sb.Length < 32) { _sb.Append(' ', 32 - _sb.Length); }

                    _sb.Append("Error");
                    if (_sb.Length < 48) { _sb.Append(' ', 48 - _sb.Length); }

                    Debug.Print(_sb.ToString()); /* print columns */
                }
            }
        }
    }
}
