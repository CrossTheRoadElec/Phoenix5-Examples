/**
 * Task manageing the CANifier outputs to the LED strip.
 */
using CTRE.Phoenix.Tasking;
using CTRE.Phoenix;
using Platform;

public class TaskMeasurePulseSensors : ILoopable
{
    float[][] _dutyCycleAndPeriods = new float[][] { new float[] { 0, 0 }, new float[] { 0, 0 }, new float[] { 0, 0 }, new float[] { 0, 0 } };

    public float GetMeasuredPulseWidthsUs(CANifier.PWMChannel pwmCh)
    {
        return _dutyCycleAndPeriods[(int)pwmCh][0];
    }

    public void OnLoop()
    {
        Hardware.canifier.GetPwmInput(CANifier.PWMChannel.PWMChannel0, _dutyCycleAndPeriods[0]);
        Hardware.canifier.GetPwmInput(CANifier.PWMChannel.PWMChannel1, _dutyCycleAndPeriods[1]);
        Hardware.canifier.GetPwmInput(CANifier.PWMChannel.PWMChannel2, _dutyCycleAndPeriods[2]);
        Hardware.canifier.GetPwmInput(CANifier.PWMChannel.PWMChannel3, _dutyCycleAndPeriods[3]);

        uint data = 0;
        data = (uint)(_dutyCycleAndPeriods[3][0] * 1000);
        CTRE.Native.CAN.Send(0x1E040000, data, 4, 0);
    }

    public override string ToString()
    {
        return "TaskMeasurePulseSensors:" +
            _dutyCycleAndPeriods[0][0] + " :" +
            _dutyCycleAndPeriods[1][0] + " :" +
            _dutyCycleAndPeriods[2][0] + " :" +
            _dutyCycleAndPeriods[3][0];
    }

    public void OnStart()
    {
        /* speed up PWM inputs */
        Hardware.canifier.SetStatusFramePeriod(CANifierStatusFrame.Status_3_PwmInputs0, 10);
        Hardware.canifier.SetStatusFramePeriod(CANifierStatusFrame.Status_4_PwmInputs1, 10);
        Hardware.canifier.SetStatusFramePeriod(CANifierStatusFrame.Status_5_PwmInputs2, 10);
        Hardware.canifier.SetStatusFramePeriod(CANifierStatusFrame.Status_6_PwmInputs3, 10);
    }
    public void OnStop() { }
    public bool IsDone() { return false; }
}
