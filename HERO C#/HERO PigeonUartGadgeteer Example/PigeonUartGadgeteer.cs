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

namespace CTRE
{
    /// <summary>
    /// Some Pigeon IMU functionality can be used by connecting Pigeon directly to any 'U' port on HERO.
    /// This lighweight class allows for polling Yaw/Pitch/Roll without the need to wire in a Talon SRX or CANbus.
    /// Pigeon will stay blinking orange, instead of the usual-green blink.
    /// </summary>
    class PigeonUartGadgeteer : CTRE.Gadgeteer.Module.ModuleBase
    {
        /* initial message to send to the terminal */
        readonly byte[] _chirpReq = { 0x5a, 0x00, 0x01, 0x10, 0x95 };
        readonly byte[] _pollMsg = { 0x5a, 0x00, 0x01, 0x16, 0x8f };

        private int _state = 0;
        private uint _len;
        private TimeScheduler _timeSched = new TimeScheduler(10);

        private byte[] _fr = new byte[14];
        private UInt64 _cache;
        private float[] _ypr = new float[3];

        SerialClient _serialClient;

        public PigeonUartGadgeteer(HERO.Port1Definition portDef)
        {
            _serialClient = new SerialClient(portDef.UART, ProcessData);
        }
        public PigeonUartGadgeteer(HERO.Port4Definition portDef)
        {
            _serialClient = new SerialClient(portDef.UART, ProcessData);
        }
        public PigeonUartGadgeteer(HERO.Port6Definition portDef)
        {
            _serialClient = new SerialClient(portDef.UART, ProcessData);
        }
        public float Yaw
        {
            get { return _ypr[0]; }
        }
        public float Pitch
        {
            get { return _ypr[1]; }
        }
        public float Roll
        {
            get { return _ypr[2]; }
        }
        private void GetThreeContinAngles(out float p1, out float p2, out float p3)
        {
            Int32 raw1, raw2, raw3;
            GetThreeParam20(out raw1, out raw2, out raw3);
            p1 = raw1 * (360f / 8192f);
            p2 = raw2 * (360f / 8192f);
            p3 = raw3 * (360f / 8192f);
        }

        private void GetThreeParam20(out Int32 p1, out Int32 p2, out Int32 p3)
        {
            byte p1_h8 = (byte)_cache;
            byte p1_m8 = (byte)(_cache >> 8);
            byte p1_l4 = (byte)(_cache >> 20);

            byte p2_h4 = (byte)(_cache >> 16);
            byte p2_m8 = (byte)(_cache >> 24);
            byte p2_l8 = (byte)(_cache >> 32);

            byte p3_h8 = (byte)(_cache >> 40);
            byte p3_m8 = (byte)(_cache >> 48);
            byte p3_l4 = (byte)(_cache >> 60);

            p1_l4 &= 0xF;
            p2_h4 &= 0xF;
            p3_l4 &= 0xF;

            p1 = p1_h8;
            p1 <<= 8;
            p1 |= p1_m8;
            p1 <<= 4;
            p1 |= p1_l4;
            p1 <<= (32 - 20);
            p1 >>= (32 - 20);

            p2 = p2_h4;
            p2 <<= 8;
            p2 |= p2_m8;
            p2 <<= 8;
            p2 |= p2_l8;
            p2 <<= (32 - 20);
            p2 >>= (32 - 20);

            p3 = p3_h8;
            p3 <<= 8;
            p3 |= p3_m8;
            p3 <<= 4;
            p3 |= p3_l4;
            p3 <<= (32 - 20);
            p3 >>= (32 - 20);
        }

        private void HandleMsg()
        {
            UInt32 arbId;
            int dlc;
            int options;

            arbId = _fr[0];
            arbId <<= 8;
            arbId |= _fr[1];
            arbId <<= 8;
            arbId |= _fr[2];
            arbId <<= 8;
            arbId |= _fr[3];

            dlc = _fr[4];

            options = _fr[5];

            _cache = _fr[13];
            _cache <<= 8;
            _cache |= _fr[12];
            _cache <<= 8;
            _cache |= _fr[11];
            _cache <<= 8;
            _cache |= _fr[10];
            _cache <<= 8;
            _cache |= _fr[9];
            _cache <<= 8;
            _cache |= _fr[8];
            _cache <<= 8;
            _cache |= _fr[7];
            _cache <<= 8;
            _cache |= _fr[6];

            arbId = arbId & 0xFFFFFFC0;

            switch (arbId)
            {
                case 0x02042200:
                    GetThreeContinAngles(out _ypr[0], out _ypr[1], out _ypr[2]);
                    break;
            }
        }
        private void ProcFrame(ByteRingBuffer ringBuffer, uint lenPlusOne)
        {
            switch (ringBuffer.Front)
            {
                case 0x10:
                    break;
                case 0x14:
                    int numFr;
                    ringBuffer.Pop();
                    numFr = ((int)lenPlusOne - 2) / 14;
                    while (numFr > 0)
                    {
                        --numFr;
                        _fr[0x0] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0x1] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0x2] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0x3] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0x4] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0x5] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0x6] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0x7] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0x8] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0x9] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0xa] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0xb] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0xc] = ringBuffer.Front; ringBuffer.Pop();
                        _fr[0xd] = ringBuffer.Front; ringBuffer.Pop();
                        HandleMsg();
                    }
                    ringBuffer.Pop();
                    lenPlusOne = 0;
                    break;
                case 0x17:
                    break;
                default:
                    break;
            }


            ringBuffer.Pop(lenPlusOne);
        }
        private void ProcessData(ByteRingBuffer ringBuffer)
        {
            /* tx tasks */
            if (_timeSched.Process())
            {
                /* every 10 ms */
                _serialClient.Write(_pollMsg);
                _serialClient.Write(_chirpReq);
            }
            /* rx process */
            bool stay = true;
            while (stay)
            {
                switch (_state)
                {
                    case 0:
                        if (ringBuffer.Count < 1)
                        {
                            stay = false; /* not enough data */
                        }
                        else
                        {
                            /* we have data */
                            if (ringBuffer.Front == 0x5a) { ++_state; }
                            /* pop it */
                            ringBuffer.Pop();
                        }
                        break;

                    case 1:
                        if (ringBuffer.Count < 2)
                        {
                            stay = false; /* not enough data */
                        }
                        else
                        {
                            byte h = ringBuffer.Front;
                            ringBuffer.Pop();
                            byte l = ringBuffer.Front;
                            ringBuffer.Pop();

                            _len = h;
                            _len <<= 8;
                            _len = l;
                            ++_state;
                        }
                        break;

                    case 2:
                        if (ringBuffer.Count < (_len + 1))
                        {
                            stay = false; /* not enough data */
                        }
                        else
                        {
                            ProcFrame(ringBuffer, _len + 1);
                            _state = 0;
                        }
                        break;
                }
            }
        }
    }
}
