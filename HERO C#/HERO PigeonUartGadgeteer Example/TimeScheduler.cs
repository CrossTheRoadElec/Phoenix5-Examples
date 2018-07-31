/*
*  Software License Agreement
*
* Copyright (C) Cross The Road Electronics.  All rights
* reserved.
* 
* Cross The Road Electronics (CTRE) licenses to you the right to 
* use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
* API Libraries ONLY when in use with Cross The Road Electronics hardware products.
* 
* THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
* WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
* LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
* CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
* INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
* PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
* BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
* THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
* SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
* (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
*/
using System;
using Microsoft.SPOT;

namespace CTRE
{
    /// <summary>
    /// General class for task scheduling.
    /// </summary>
    public class TimeScheduler
    {
        long _periodMs; //!< Holds the target time period to fire scheduled events
        long _lastTime = 0; //!< Time at which last event fired.
        long _tenTimesPeriodMs; //!< 10X the target time period, if we fall to far ahead or behind, rearm everything cleanly to robustly protect against clock resets.
        bool _error = false; //!< Unused flag for debugging purposes

        public TimeScheduler(long periodMs)
        {
            if (periodMs < 1)
                periodMs = 1;

            _periodMs = periodMs;
            Reset();
        }
        void Reset()
        {
            _tenTimesPeriodMs = 10 * _periodMs;
            _lastTime = GetMs();
        }
        long GetMs()
        {
            long now = DateTime.Now.Ticks;
            now /= 10000; //100ns per unit
            return now;
        }

        public bool Process()
        {
            bool retval = false;

            long t = GetMs();
            long delta = t - _lastTime;
            if ((delta > _tenTimesPeriodMs) || (delta < -_tenTimesPeriodMs))
            {
                /* something is wrong, signal a time out, and rearm cleanly for next timeout */
                _lastTime = GetMs();
                retval = true;
                /* for now note the event, wire this to something if timing issues ensue */
                _error = true;
                _error = (_error == true); // removes unused warning
            }
            else if (delta >= _periodMs)
            {
                /* move up last time by one timeout period, this assumes we never fall to far behind, also this gives us ideal mean time */
                _lastTime += _periodMs;
                retval = true;
            }

            return retval;
        }
    }
}
