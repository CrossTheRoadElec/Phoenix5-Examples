/**
 * Sync quadrature CTRE Mag encoder to it's absolute value and offset to remove discountinuities.
 * @link http://www.ctr-electronics.com/srx-magnetic-encoder.html
 */

public class TaskOnceAbsoluteArmSensor : CTRE.Phoenix.Tasking.ILoopable
{
    /* ILoopable */
    public bool IsDone()
    {
        return true;
    }

    public void OnLoop()
    {
        /* do nothing, we just need to setup our sensor on boot */
    }

    public void OnStart()
    {
        /* make 3276 our new zero so that the discontinity in the CTRE mag encoder is removed */
        RemoveDiscontinuityMagEncoder.Offset(Platform.Hardware.armTalon, true, Platform.Constants.ArmAbsoluteSensorOffset);
    }

    public void OnStop()
    {
    }
}