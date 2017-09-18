using System;
using Microsoft.SPOT;

namespace HERO_Mecanum_Drive_Example.Platform
{
    public static class Tasks
    {
        /* Subsystem tasks.  A task could be all the functionality of a substem like a robot arm, 
         * or could be an individual action appy to a subsytem, such as up reading a gamepad and applying it to the drivcetrain.
         * Use 'public static' because these are single objects. 
         */

        public static TaskEnableRobot EnableRobot = new TaskEnableRobot();
        public static TaskLEDStrip LEDStrip = new TaskLEDStrip();
        public static TaskTeleopDriveWithRC TeleopDriveWithRC = new TaskTeleopDriveWithRC();
        public static TaskTeleopDriveWithXbox TeleopDriveWithXbox = new TaskTeleopDriveWithXbox();
        public static TaskLowBatteryDetect LowBatteryDetect = new TaskLowBatteryDetect();

        
    }
}
