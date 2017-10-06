using System;
using Microsoft.SPOT;

namespace CTRE
{
    public class LowLevel_CANifier : CANBusDevice
    {
        const UInt32 STATUS_1 = 0x03041400;
        const UInt32 STATUS_2 = 0x03041440;
        const UInt32 STATUS_3 = 0x03041480;
        const UInt32 STATUS_4 = 0x030414C0;
        const UInt32 STATUS_5 = 0x03041500;
        const UInt32 STATUS_6 = 0x03041540;
        const UInt32 STATUS_7 = 0x03041580;
        const UInt32 STATUS_8 = 0x030415C0;
        const UInt32 STATUS_9 = 0x03041600;

        const UInt32 CANifier_Control_1_General_20Ms = 0x03040000;
        const UInt32 CANifier_Control_2_PwmOutput = 0x03040040;

        const UInt32 PARAM_REQUEST = 0x03041800;
        const UInt32 PARAM_RESPONSE = 0x03041840;
        const UInt32 PARAM_SET = 0x03041880;

        const UInt32 kParamArbIdValue = PARAM_RESPONSE;
        const UInt32 kParamArbIdMask = 0xFFFFFFFF;

        private UInt64 _cache;
        private UInt32 _len;

        private UInt32 _can_h = 0;
        private int _can_stat = 0;
        private System.Collections.Hashtable _sigs = new System.Collections.Hashtable();

        private uint _regInput = 0; //!< Decoded inputs
        private uint _regLat = 0; //!< Decoded output latch
        private uint _regIsOutput = 0; //!< Decoded data direction register

        private int _lastError = 0;

        public enum GeneralPin
        {
            QUAD_IDX = (0),
            QUAD_B = (1),
            QUAD_A = (2),
            LIMR = (3),
            LIMF = (4),
            SDA = (5),
            SCL = (6),
            SPI_CS = (7),
            SPI_MISO_PWM2P = (8),
            SPI_MOSI_PWM1P = (9),
            SPI_CLK_PWM0P = (10),
        }

        private const int kTotalGeneralPins = 11;

        bool _SendingPwmOutput = false;

        public LowLevel_CANifier(UInt16 deviceId, bool externalEnable = false) : base(deviceId)
        {
            if (false == externalEnable)
            {
                CTRE.Native.CAN.Send(CANifier_Control_1_General_20Ms | _deviceNumber, 0x00, 8, 20);
            }
        }

        private void EnsurePwmOutputFrameIsTransmitting()
        {
            if (false == _SendingPwmOutput)
            {
                _SendingPwmOutput = true;

                ulong frame = 0x0000000000000000;
                CTRE.Native.CAN.Send(CANifier_Control_2_PwmOutput | _deviceNumber, frame, 8, 10);
            }
        }

        private static int DecodeRegLat(ulong frame)
        {
            byte b5 = (byte)(frame >> 40);
            byte b6 = (byte)(frame >> 48);
            int retval = b5;
            retval <<= 3;
            retval |= b6 >> 5;
            return retval;
        }
        private static int DecodeRegIsOutput(ulong frame)
        {
            byte b6 = (byte)(frame >> 48);
            byte b7 = (byte)(frame >> 56);
            int retval = b6 & 0x7;
            retval <<= 8;
            retval |= b7;
            return retval;
        }
        private static int DecodeRegInput(ulong frame)
        {
            byte b5 = (byte)(frame >> 40);
            byte b6 = (byte)(frame >> 48);
            int retval = b5;
            retval &= 0x7;
            retval <<= 8;
            retval |= b6;
            return retval;
        }

        private static void Set10Bitvalue(int value_10, int position_0123, ref UInt64 frame)
        {
            /* serialize parts */
            int C0_h8 = value_10 >> 2;
            int C0_l2 = value_10;
            int C1_h6 = value_10 >> 4;
            int C1_l4 = value_10;
            int C2_h4 = value_10 >> 6;
            int C2_l6 = value_10;
            int C3_h2 = value_10 >> 8;
            int C3_l8 = value_10;
            C0_h8 &= 0xFF;
            C0_l2 &= 0x03;
            C1_h6 &= 0x3F;
            C1_l4 &= 0x0F;
            C2_h4 &= 0x0F;
            C2_l6 &= 0x3F;
            C3_h2 &= 0x03;
            C3_l8 &= 0xFF;
            /* serialize it */
            switch (position_0123)
            {
                case 0:
                    frame &= ~(0xFFul);
                    frame &= ~(0x03ul << (8 + 6));
                    frame |= (UInt64)(byte)(C0_h8);
                    frame |= (UInt64)(C0_l2) << (8 + 6);
                    break;
                case 1:
                    frame &= ~(0x3Ful << 8);
                    frame &= ~(0xFul << (16 + 4));
                    frame |= (UInt64)(C1_h6) << 8;
                    frame |= (UInt64)(C1_l4) << (16 + 4);
                    break;
                case 2:
                    frame &= ~(0x0Ful << 16);
                    frame &= ~(0x3Ful << (24 + 2));
                    frame |= (UInt64)(C2_h4) << 16;
                    frame |= (UInt64)(C2_l6) << (24 + 2);
                    break;
                case 3:
                    frame &= ~(0x03ul << 24);
                    frame &= ~(0xFFul << (32));
                    frame |= (UInt64)(C3_h2) << 24;
                    frame |= (UInt64)(C3_l8) << (32);
                    break;
            }
        }

        private int HandleError(int error)
        {
            _lastError = error;
            return error;
        }

        public int SetLEDOutput(int dutyCycle, uint ledChannel)
        {
            int retval = CTRE.Native.CAN.GetSendBuffer(CANifier_Control_1_General_20Ms | _deviceNumber, ref _cache);
            if (retval != 0)
                return retval;
            /* serialize it */
            switch (ledChannel)
            {
                case 0:
                    Set10Bitvalue(dutyCycle, 0, ref _cache);
                    break;
                case 1:
                    Set10Bitvalue(dutyCycle, 1, ref _cache);
                    break;
                case 2:
                    Set10Bitvalue(dutyCycle, 2, ref _cache);
                    break;
            }
            /* save it */
            CTRE.Native.CAN.Send(CANifier_Control_1_General_20Ms | _deviceNumber, _cache, 8, 0xFFFFFFFF);
            return HandleError(retval);
        }
        public int SetGeneralOutputs(UInt32 outputsBits, UInt32 isOutputBits)
        {
            /* sterilize inputs */
            outputsBits &= 0x7FF;
            isOutputBits &= 0x7FF;
            /* save values into registers*/
            _regLat = outputsBits;
            _regIsOutput = isOutputBits;
            /* get tx message */
            int retval = CTRE.Native.CAN.GetSendBuffer(CANifier_Control_1_General_20Ms | _deviceNumber, ref _cache);
            if (retval != 0)
                return retval;
            /* calc bytes 5,6,7 */
            byte b5 = (byte)(_regLat >> 3);
            byte b6 = (byte)((_regLat & 7) << 5);
            b6 |= (byte)(_regIsOutput >> 8);
            byte b7 = (byte)(_regIsOutput);

            _cache &= ~0xFFFFFF0000000000;
            _cache |= ((ulong)b5) << 40;
            _cache |= ((ulong)b6) << 48;
            _cache |= ((ulong)b7) << 56;

            /* save it */
            CTRE.Native.CAN.Send(CANifier_Control_1_General_20Ms | _deviceNumber, _cache, 8, 0xFFFFFFFF);
            return HandleError(retval);

        }
        public int SetGeneralOutput(GeneralPin outputPin, bool outputValue, bool outputEnable)
        {
            /* calc bitpos from caller's selected ch */
            uint mask = (uint)1 << (int)outputPin;

            /* update regs based on caller's params */
            if (outputValue)
                _regLat |= mask;
            else
                _regLat &= ~mask;

            if (outputEnable)
                _regIsOutput |= mask;
            else
                _regIsOutput &= ~mask;

            return HandleError(SetGeneralOutputs(_regLat, _regIsOutput));
        }

        public int SetPWMOutput(uint pwmChannel, int dutyCycle)
        {
            /* start transmitting if not done yet */
            EnsurePwmOutputFrameIsTransmitting();

            int retval = CTRE.Native.CAN.GetSendBuffer(CANifier_Control_2_PwmOutput | _deviceNumber, ref _cache);
            if (retval != 0)
                return retval;
            /* serialize it */
            switch (pwmChannel)
            {
                case 0:
                    Set10Bitvalue(dutyCycle, 0, ref _cache);
                    break;
                case 1:
                    Set10Bitvalue(dutyCycle, 1, ref _cache);
                    break;
                case 2:
                    Set10Bitvalue(dutyCycle, 2, ref _cache);
                    break;
                case 3:
                    Set10Bitvalue(dutyCycle, 3, ref _cache);
                    break;
            }
            /* save it */
            CTRE.Native.CAN.Send(CANifier_Control_2_PwmOutput | _deviceNumber, _cache, 8, 0xFFFFFFFF);
            return HandleError(retval);
        }
        private byte SetBit(byte value, int bitpos)
        {
            int ret =  value | (1 << bitpos);
            return (byte)ret;
        }
        private byte ClrBit(byte value, int bitpos)
        {
            int ret = value & ~(1 << bitpos);
            return (byte)ret;
        }
        private byte SetClrBit(byte value, int bitpos, bool bSet)
        {
            if (bSet) return SetBit(value, bitpos);
            return ClrBit(value, bitpos);
        }

        public int EnablePWMOutput(uint pwmChannel, bool bEnable)
        {
            /* start transmitting if not done yet */
            EnsurePwmOutputFrameIsTransmitting();

            int retval = CTRE.Native.CAN.GetSendBuffer(CANifier_Control_2_PwmOutput | _deviceNumber, ref _cache);
            if (retval != 0)
                return retval;

            byte b7 = (byte)(_cache >> 56);

            /* serialize it */
            switch (pwmChannel)
            {
                case 0:
                case 1:
                case 2:
                case 3:
                    b7 = SetClrBit(b7, 4 + (int)pwmChannel, bEnable);
                    break;
            }
            /* save it */
            _cache &= ~0xFF00000000000000;
            _cache |= ((ulong)b7 << 56);
            CTRE.Native.CAN.Send(CANifier_Control_2_PwmOutput | _deviceNumber, _cache, 8, 0xFFFFFFFF);
            return HandleError(retval);
        }
        public uint GetGeneralInputs()
        {
            int err = CTRE.Native.CAN.Receive(STATUS_2 | _deviceNumber, ref _cache, ref _len);

            _regInput = (uint)DecodeRegInput(_cache);

            HandleError(err);
            return _regInput;
        }

        public int GetGeneralInputs(int [] allPins)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_2 | _deviceNumber, ref _cache, ref _len);

            _regInput = (uint)DecodeRegInput(_cache);

            for (int i=0;i< kTotalGeneralPins;++i)
            {
                int mask = 1 << i;
                if (((_regInput & mask) != 0))
                {
                    allPins[i] = 1;
                } else
                {
                    allPins[i] = 0;
                }
            }
            return HandleError(retval);
        }
        public bool GetGeneralInput(GeneralPin inputPin)
        {
            int retval = CTRE.Native.CAN.Receive(STATUS_2 | _deviceNumber, ref _cache, ref _len);
            HandleError(retval);

            int mask = 1 << (int)inputPin;
            _regInput = (uint)DecodeRegInput(_cache);

            if (((_regInput & mask) != 0))
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        private int GetPwmInput_X(uint ArbIDField, uint [] waveform)
        {
            uint pulseWidthFrac;
            uint periodRaw;
            int retval = CTRE.Native.CAN.Receive(ArbIDField | _deviceNumber, ref _cache, ref _len);
            byte b0 = (byte)(_cache >> 0x00);
            byte b1 = (byte)(_cache >> 0x08);
            byte b2 = (byte)(_cache >> 0x10);
            byte b3 = (byte)(_cache >> 0x18);
            byte b4 = (byte)(_cache >> 0x20);
            byte b5= (byte)(_cache >> 0x28);

            pulseWidthFrac = b0;
            pulseWidthFrac <<= 8;
            pulseWidthFrac |= b1;
            pulseWidthFrac <<= 8;
            pulseWidthFrac |= b2;

            periodRaw = b3;
            periodRaw <<= 8;
            periodRaw |= b4;
            periodRaw <<= 3;
            periodRaw |= (byte)(b5 >> 5);

            waveform[0] = pulseWidthFrac;
            waveform[1] = periodRaw;

            return retval;
        }
        public int GetPwmInput(uint pwmChannel, float [] dutyCycleAndPeriod)
        {
            uint[] temp = new uint[2] { 0, 0 };
            int retval = (int)Codes.CAN_INVALID_PARAM;

            float pulseWidthUs;
            float periodUs;

            switch (pwmChannel)
            {
                case 0:
                    retval = GetPwmInput_X(STATUS_3, temp); // ref pulseWidthFrac, ref periodRaw);
                    break;
                case 1:
                    retval = GetPwmInput_X(STATUS_4, temp); //ref pulseWidthFrac, ref periodRaw);
                    break;
                case 2:
                    retval = GetPwmInput_X(STATUS_5, temp); // ref pulseWidthFrac, ref periodRaw);
                    break;
                case 3:
                    retval = GetPwmInput_X(STATUS_6, temp); //ref pulseWidthFrac, ref periodRaw);
                    break;
            }


            /* scaling */
            uint pulseWidthFrac = temp[0], periodRaw = temp[1];
            periodUs = periodRaw * 0.256f; /* convert to microseconds */
            pulseWidthUs = pulseWidthFrac * 0.000244140625f * periodUs;

            dutyCycleAndPeriod[0] = pulseWidthUs;
            dutyCycleAndPeriod[1] = periodUs;

            return HandleError(retval);
        }

        public int GetLastError()
        {
            return _lastError;
        }


        public float GetBatteryVoltage()
        {
            int err = CTRE.Native.CAN.Receive(STATUS_1 | _deviceNumber, ref _cache, ref _len);

            byte b5 = (byte)(_cache >> 40);

            HandleError(err);
            return b5 * 0.1f + 4f;
        }

        public static String ToString(LowLevel_CANifier.GeneralPin gp)
        {
            String sig;
            switch (gp)
            {
                case LowLevel_CANifier.GeneralPin.LIMR: sig = "LIMR"; break;
                case LowLevel_CANifier.GeneralPin.LIMF: sig = "LIMF"; break;
                case LowLevel_CANifier.GeneralPin.QUAD_IDX: sig = "QUAD_IDX"; break;
                case LowLevel_CANifier.GeneralPin.QUAD_B: sig = "QUAD_B"; break;
                case LowLevel_CANifier.GeneralPin.QUAD_A: sig = "QUAD_A"; break;
                case LowLevel_CANifier.GeneralPin.SDA: sig = "SDA"; break;
                case LowLevel_CANifier.GeneralPin.SCL: sig = "SCL"; break;
                case LowLevel_CANifier.GeneralPin.SPI_CS: sig = "SPI_CS"; break;
                case LowLevel_CANifier.GeneralPin.SPI_MISO_PWM2P: sig = "SPI_MISO_PWM2P"; break;
                case LowLevel_CANifier.GeneralPin.SPI_MOSI_PWM1P: sig = "SPI_MOSI_PWM1P"; break;
                case LowLevel_CANifier.GeneralPin.SPI_CLK_PWM0P: sig = "SPI_CLK_PWM0P"; break;
                default: sig = "invalid"; break;
            }
            return sig;
        }

        //----------------------------------------------------------------------------------------------------//
        /**
        * Signal enumeration for generic signal access.
        * Although every signal is enumerated, only use this for traffic that must
        * be solicited.
        * Use the auto generated getters/setters at bottom of this header as much as
        * possible.
        */
        public enum ParamEnum
        {
            eStatusFrameRate = 200,
        };


        private void OpenSessionIfNeedBe()
        {
            _can_stat = 0;
            if (_can_h == 0)
            {
                /* bit30 - bit8 must match $000002XX.  Top bit is not masked to get remote frames */
                uint arbId = kParamArbIdValue | GetDeviceNumber();
                _can_stat = CTRE.Native.CAN.OpenStream(ref _can_h, kParamArbIdMask, arbId);
                if (_can_stat == 0)
                {
                    /* success */
                }
                else
                {
                    /* something went wrong, try again later */
                    _can_h = 0;
                }
            }
        }

        private void ProcessStreamMessages()
        {
            if (0 == _can_h) OpenSessionIfNeedBe();
            /* process receive messages */
            UInt32 i;
            UInt32 messagesRead = 0;
            UInt32 arbId = 0;
            UInt64 data = 0;
            UInt32 len = 0;
            UInt32 msgsRead = 0;
            /* read out latest bunch of messages */
            _can_stat = 0;
            if (_can_h != 0)
            {
                CTRE.Native.CAN.GetStreamSize(_can_h, ref messagesRead);
            }
            /* loop thru each message of interest */
            for (i = 0; i < messagesRead; ++i)
            {
                CTRE.Native.CAN.ReadStream(_can_h, ref arbId, ref data, ref len, ref msgsRead);
                if (arbId == (PARAM_RESPONSE | GetDeviceNumber()))
                {
                    byte paramEnum = (byte)(data & 0xFF);
                    data >>= 8;
                    /* save latest signal */
                    _sigs[(uint)paramEnum] = (uint)data;
                }
            }
        }
        /*---------------------setters and getters that use the param
         * request/response-------------*/
        /**
         * Send a one shot frame to set an arbitrary signal.
         * Most signals are in the control frame so avoid using this API unless you have
         * to.
         * Use this api for...
         * -A motor controller profile signal eProfileParam_XXXs.  These are backed up
         * in flash.  If you are gain-scheduling then call this periodically.
         * -Default brake and limit switch signals... eOnBoot_XXXs.  Avoid doing this,
         * use the override signals in the control frame.
         * Talon will automatically send a PARAM_RESPONSE after the set, so
         * GetParamResponse will catch the latest value after a couple ms.
         */
        public int SetParamRaw(ParamEnum paramEnum, int rawBits, uint timeoutMs = 0)
        {
            /* caller is using param API.  Open session if it hasn'T been done. */
            if (0 == _can_h) OpenSessionIfNeedBe();
            /* wait for response frame */
            if (timeoutMs != 0)
            {
                /* remove stale entry if caller wants to wait for response. */
                _sigs.Remove((uint)paramEnum);
            }
            /* frame set request and send it */
            UInt64 frame = ((UInt64)rawBits) & 0xFFFFFFFF;
            frame <<= 8;
            frame |= (byte)paramEnum;
            uint arbId = PARAM_SET | GetDeviceNumber();
            int status = CTRE.Native.CAN.Send(arbId, frame, 5, 0);
            /* wait for response frame */
            if (timeoutMs > 0)
            {
                int readBits;
                /* loop until timeout or receive if caller wants to check */
                while (timeoutMs > 0)
                {
                    /* wait a bit */
                    System.Threading.Thread.Sleep(1);
                    /* see if response was received */
                    if (0 == GetParamResponseRaw(paramEnum, out readBits))
                        break; /* leave inner loop */
                    /* decrement */
                    --timeoutMs;
                }
                /* if we get here then we timed out */
                if (timeoutMs == 0)
                    status = (int)Codes.CTR_SigNotUpdated;
            }
            return status;
        }
        /**
         * Checks cached CAN frames and updating solicited signals.
         */
        public int GetParamResponseRaw(ParamEnum paramEnum, out Int32 rawBits)
        {
            int retval = 0;
            /* process received param events. We don't expect many since this API is not
             * used often. */
            ProcessStreamMessages();
            /* grab the solicited signal value */
            if (_sigs.Contains((uint)paramEnum) == false)
            {
                retval = (int)Codes.CTR_SigNotUpdated;
                rawBits = 0; /* default value if signal was not received */
            }
            else
            {
                Object value = _sigs[(uint)paramEnum];
                uint temp = (uint)value;
                rawBits = (int)temp;
            }
            return retval;
        }
        /**
         * Asks TALON to immedietely respond with signal value.  This API is only used
         * for signals that are not sent periodically.
         * This can be useful for reading params that rarely change like Limit Switch
         * settings and PIDF values.
          * @param param to request.
         */
        public int RequestParam(ParamEnum paramEnum)
        {
            /* process received param events. We don't expect many since this API is not
             * used often. */
            ProcessStreamMessages();
            int status = CTRE.Native.CAN.Send(PARAM_REQUEST | GetDeviceNumber(), (uint)paramEnum, 1, 0);
            return status;
        }

        public int SetParam(ParamEnum paramEnum, float value, uint timeoutMs = 0)
        {
            Int32 rawbits = 0;
            switch (paramEnum)
            {
                default: /* everything else is integral */
                    rawbits = (Int32)value;
                    break;
            }
            return SetParamRaw(paramEnum, rawbits, timeoutMs);
        }
        public int GetParamResponse(ParamEnum paramEnum, out float value)
        {
            Int32 rawbits = 0;
            int retval = GetParamResponseRaw(paramEnum, out rawbits);
            switch (paramEnum)
            {
                default: /* everything else is integral */
                    value = (float)rawbits;
                    break;
            }
            return retval;
        }
        public Int32 GetParamResponseInt32(ParamEnum paramEnum, out int value)
        {
            float dvalue = 0;
            int retval = GetParamResponse(paramEnum, out dvalue);
            value = (Int32)dvalue;
            return retval;
        }
        /*----- getters and setters that use param request/response. These signals are backed up in flash and will survive a power cycle. ---------*/
        /*----- If your application requires changing these values consider using both slots and switch between slot0 <=> slot1. ------------------*/
        /*----- If your application requires changing these signals frequently then it makes sense to leverage this API. --------------------------*/
        /*----- Getters don't block, so it may require several calls to get the latest value. --------------------------*/
        public int SetStatusFrameRate(UInt32 frameIdx, uint newPeriodMs, uint timeoutMs = 0)
        {
            if (newPeriodMs > 255) { newPeriodMs = 255; }
            if (frameIdx > 255) { frameIdx = 255; }

            UInt32 value = newPeriodMs;
            value <<= 8;
            value |= (frameIdx & 0xFF);

            return SetParam(ParamEnum.eStatusFrameRate, value, timeoutMs);
        }
    }
}
