/**
 * Helpful function to remove discountinuity from a CTRE Mag encoder.
 * When using an absolute sensor.
 * 
 * You may find the overflow discontinuity of an absolute CTRE Mag encoder
 * is in the functional range of your mechanism.  An easy solution is to
 * set the Talon to use the relative (quadrature) signals of the CTRE Mag
 * encoder, and seed it to match the absolute (pulse width) on boot with an
 * offset to ensure it has no discontinuity.  This routine will accomplish this.
 * 
 * This accomplishes
 * - The sensor will power boot to the same absolute sensor value with no need to home
 *  the sensor.  This assumes the sensor only rotates within one rotation during use.
 * - The sensor has no discountinuities, no need to adjust sensor to accomplish this.
 * - Since quadrature is used after boot, sensor value updates are fast (not limited by pulse width update rate).
 * 
 * Long term this will be integrate into Talon firmware in the final Phoenix Framework.
 **/
using CTRE.Phoenix.MotorControl.CAN;

public class RemoveDiscontinuityMagEncoder
{
    public static void Offset(TalonSRX talon, bool sensorIsReversed, int offset)
    {
		/* read the talon's absolute pulse wid */
		Platform.Hardware.armTalon.ConfigSelectedFeedbackSensor(CTRE.Phoenix.MotorControl.FeedbackDevice.PulseWidthEncodedPosition);
        int pos = Platform.Hardware.armTalon.GetSelectedSensorPosition();
		/* keep bottom 12bits */
		pos &= 0xFFF;
        /* offset */
        {
            pos -= offset;
            if (pos < 0)
                pos += 4096;
        }
        /* redo negative */
        if(sensorIsReversed)
            pos *= -1;

        /* set it back */
        Platform.Hardware.armTalon.SetSelectedSensorPosition(pos);
    }
}
