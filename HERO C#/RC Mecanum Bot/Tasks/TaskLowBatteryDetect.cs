/**
 * Task tracking the robot battery voltage.  If the battery voltage is considered low,
 * other subsystem tasks will cut the motor speed to signal user to change the battery.
 */

namespace HERO_Mecanum_Drive_Example
{
    public class TaskLowBatteryDetect : CTRE.Phoenix.Tasking.ILoopable
    {
        int _dnCnt = 0;
        int _upCnt = 0;

        public bool BatteryIsLow { get; private set;}
      
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
            vbat += Platform.Hardware.leftFrnt.GetBusVoltage();
            vbat += Platform.Hardware.rghtFrnt.GetBusVoltage();
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
        }

        public void OnStart()
        {
        }

        public void OnStop()
        {
        }
    }
}
