/**
 * Decodes the PWM inputs from CANifier for the Futaba RC Radio.
 * Tested range exceeds 400ft.
 * @link http://www.futabarc.com/systems/futk3100.html
 */
namespace CTRE
{
    public class RCRadio_Futaba3Ch : IProcessable
    {
        private CANifier _canifier;

        private float[][] _dutyCycleAndPeriods = {
            new float[] { 0, 0 },
            new float[] { 0, 0 },
            new float[] { 0, 0 },
            new float[] { 0, 0 }
        };

        private int[] _errorCodes = new int[4];

        public enum Channel
        {
            Channel1,
            Channel2,
            Channel3,
        }

        public enum Status
        {
            LossOfCAN,
            LossOfPwm,
            Ok,
        }

        public RCRadio_Futaba3Ch(CANifier canifier)
        {
            _canifier = canifier;

            _canifier.SetStatusFrameRate(CANifier.StatusFrameRate.Status3_PwmInput0, 8, 100);
            _canifier.SetStatusFrameRate(CANifier.StatusFrameRate.Status4_PwmInput1, 8, 100);
            _canifier.SetStatusFrameRate(CANifier.StatusFrameRate.Status5_PwmInput2, 8, 100);
            _canifier.SetStatusFrameRate(CANifier.StatusFrameRate.Status6_PwmInput3, 8, 100);
        }

        public Status CurrentStatus { get; private set; }

        public float GetDutyCycleUs(Channel channel)
        {
            return _dutyCycleAndPeriods[(int)channel][0];
        }

        public float GetDutyCyclePerc(Channel channel)
        {
            float retval = GetDutyCycleUs(channel);

            retval = LinearInterpolation.Calculate(retval, 1000, -1, 2000, 1);

            if (retval < -1)
            {
                retval = -1;
            }
            else if (retval > +1)
            {
                retval = +1;
            }

            return retval;
        }

        public bool GetSwitchValue(Channel channel)
        {
            float retval = GetDutyCyclePerc(channel);

            return retval > 0.5f;
        }

        public float GetPeriodUs(Channel channel)
        {
            return _dutyCycleAndPeriods[(int)channel][1];
        }


        public void Process()
        {
            _errorCodes[0] = _canifier.GetPwmInput(CTRE.CANifier.PWMChannel.PWMChannel0, _dutyCycleAndPeriods[0]);
            _errorCodes[1] = _canifier.GetPwmInput(CTRE.CANifier.PWMChannel.PWMChannel1, _dutyCycleAndPeriods[1]);
            _errorCodes[2] = _canifier.GetPwmInput(CTRE.CANifier.PWMChannel.PWMChannel2, _dutyCycleAndPeriods[2]);
            _errorCodes[3] = _canifier.GetPwmInput(CTRE.CANifier.PWMChannel.PWMChannel3, _dutyCycleAndPeriods[3]);


            Status health = Status.Ok;
            if (health == Status.Ok)
            {
                /* loss of CAN means CANifier is not present */
                if (_errorCodes[0] < 0) { health = Status.LossOfCAN; }
                if (_errorCodes[1] < 0) { health = Status.LossOfCAN; }
                if (_errorCodes[2] < 0) { health = Status.LossOfCAN; }
                if (_errorCodes[3] < 0) { health = Status.LossOfCAN; }
            }

            if (health == Status.Ok)
            {
                /* loss of PWM signals means loss of radio or transceiver*/
                if (GetPeriodUs(Channel.Channel1) == 0) { health = Status.LossOfPwm; }
                if (GetPeriodUs(Channel.Channel2) == 0) { health = Status.LossOfPwm; }
                if (GetPeriodUs(Channel.Channel3) == 0) { health = Status.LossOfPwm; }
            }
            CurrentStatus = health; /* save it for calling app */
        }


        public override string ToString()
        {
            System.String retval = "";

            retval += CurrentStatus;
            retval += ",";
            retval += GetDutyCyclePerc(Channel.Channel1);
            retval += ",";
            retval += GetDutyCyclePerc(Channel.Channel2);
            retval += ",";
            retval += GetSwitchValue(Channel.Channel3);
            retval += ",";

            return retval;
        }

    }
}
