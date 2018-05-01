/*
 *  Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) binary firmware files (*.crf) 
 * and software example source ONLY when in use with Cross The Road Electronics hardware products.
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
using CTRE.Phoenix.MotorControl.CAN;

namespace Hero_PixyDrive
{
    public class Battery
    {
        TalonSRX _talon;
        int _dnCnt = 0;
        int _upCnt = 0;
        bool batIsLow = false;

        public Battery (TalonSRX talon)
        {
            _talon = talon;
        }
        public bool IsLow()
        {
            float vbat;

            vbat = _talon.GetBusVoltage();

            if (vbat > 10.50)
            {
                _dnCnt = 0;
                if (_upCnt < 100)
                    ++_upCnt;
            }
            else if (vbat < 10.00)
            {
                _upCnt = 0;
                if (_dnCnt < 100)
                    ++_dnCnt;
            }

            if (_dnCnt > 50)
                batIsLow = true;
            else if (_upCnt > 50)
                batIsLow = false;
            else
            {
                //don't change filter ouput
            }
            return batIsLow;
        }

    }
}
