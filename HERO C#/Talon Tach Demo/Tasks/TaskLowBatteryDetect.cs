/**
 * Task tracking the robot battery voltage.  
 * 
 * If the battery voltage is considered low, consider signaling the user of this.
 * Especially if using battery chemistries that require a low voltage lock out.
 * Example ways are to...
 * - severly reduce the drivetrain output so user is made aware (not recommended for FRC robots).
 * - print statements
 * - CTRE Display @link http://www.ctr-electronics.com/gadgeteer-display-module.html
 * - LED strip update (CANifier). @link http://www.ctr-electronics.com/can-can-canifier-driver-led-driver-gpio.html
 */

public class TaskLowBatteryDetect : CTRE.Phoenix.Tasking.ILoopable
{
    int _dnCnt = 0;
    int _upCnt = 0;

    public bool BatteryIsLow { get; private set; }
    public float BatteryVoltage { get; private set; }

    /* ILoopable */
    public bool IsDone()
    {
        return false;
    }

    public void OnLoop()
    {
        float vbat;

        /* get the average voltage from a couple talons */
        vbat = 0;
        vbat += Platform.Hardware.armTalon.GetBusVoltage();
        vbat += Platform.Hardware.wheelTalon.GetBusVoltage();
        vbat *= 0.5f;

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
        {
            BatteryIsLow = true;
        }
        else if (_upCnt > 50)
        {
            BatteryIsLow = false;
        }
        else
        {
            //don't change filter ouput
        }

        BatteryVoltage = vbat;
    }

    public void OnStart()
    {
    }

    public void OnStop()
    {
    }

    public override string ToString()
    {
        if (BatteryIsLow)
            return "Batt:Low VBAT:" + BatteryVoltage;
        return "Batt:" + BatteryVoltage;
    }
}
