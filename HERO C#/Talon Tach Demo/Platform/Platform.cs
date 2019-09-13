/**
 * All robot platforms have common features that should be listed below.
 * 
 * Typical examples are...
 * - System wide constants that make configuration of your robot simple
 * - List of all hardware components including gearboxes, linkages, motor controllers, etc.
 * - Schedulers to encoder list of manuevers or general round robin tasks.
 * - Subsystems, class containing custom implementations of actuator-sensor-control or high level implementatoin of multi-motor/multi-sensors.
 *   Servo* classes in Phoenix framework will likely replace this for most use cases.
 * - Tasks (commands) meant to executed periodically or in a specific order.  
 *   Tasks can be stopped/started by other tasks or high level logic via the global scheduler(s).
 * 
 * These groups can be split up into multiple modules, or in one Platform file.
 * 
 * Long term there will likely be a configurable GUI in Phoenix Framework to create robot applciations in this format (in multiple languages).
 */

using CTRE.Phoenix.Tasking;
using CTRE.Phoenix.Controller;
using CTRE.HERO;
using CTRE.Phoenix.Mechanical;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Gadgeteer.Module;
namespace Platform
{
    /* System wide constants.  Use 'public static' because these are single objects. */
    public static class Constants
    {
        /* when the ARM is at the top position, the pulse width raw signal is 3276.
         * At the bottom the sensor is about 327.
         * So offset by 3276 so it absolute signal does not wrap.
         */
        public static int ArmAbsoluteSensorOffset = 3276;

        /* ARM gains */
        public static float KPARM = 1f; //1.9f;
        public static float KDARM = 0 * 23.5f;
        public static float KIARM = 0 * 0.001f;
        /* peak voltage for ARM */
        public static float MAX_VOLTAGE = 8.9f;
        /* tolerance for close loop err */
        public static int TOLERANCE = 0;
        /* ARM target positions */
        public static float Target1 = -0.1f;
        public static float Target2 = -0.23f;

        /* WHEEL target speeds in RPM */
        public static float SpeedTarget1 = 150;
        public static float SpeedTarget2 = 200;

        /* WHEEL white marks per rotation. Wheel is a six spoke 6" Rubber wheel. Each spoke is paint markered white. */
        public static float MarksPerRotation = 6; //!< We have six white marks on our Tach/Wheel.
    }

    /* The hardware objects.  Use 'public static' because these are single objects. */
    public static class Hardware
    {
        /* Talons on CAN bus */
        public static TalonSRX armTalon = new TalonSRX(2); //Talon ID = 1,
        public static TalonSRX wheelTalon = new TalonSRX(0); //Talon ID = 0
        /* logitech gamepad */
        public static GameController gamepad = new CTRE.Phoenix.Controller.GameController(CTRE.Phoenix.UsbHostDevice.GetInstance(0), 0);
        /* sensor slice is the last stage before geared output */
        public static VersaPlanetaryWithMagEnc ArmGearBox = new VersaPlanetaryWithMagEnc(4096f,Hardware.armTalon);
        /* no gearbox, just 1:1 linkage */
        public static Gearbox WheelGearBox = new Gearbox(Hardware.wheelTalon);
        /* CTRE LCD display */
        public static DisplayModule displayModule = new DisplayModule(CTRE.HERO.IO.Port8, DisplayModule.OrientationType.Portrait_UpsideDown);
    }

    public static class Schedulers
    {
        /* the schedulers.  Minimally you will likely want one periodic scheduler to run the normal tasks.
         * Additional schedulers could be ConsecutiveSchedulers for entire autonomous movements or preconfigured manuevers.
         * Use 'public static' because these are single objects. */

        public static ConcurrentScheduler PeriodicTasks = new ConcurrentScheduler(10);
    }

    public static class Subsystems
    {
        public static Subsystem.SubSystemArm Arm = new Subsystem.SubSystemArm();
        public static Subsystem.SubSystemWheel Wheel = new Subsystem.SubSystemWheel();
    }

    public static class Tasks
    {
        /* Subsystem tasks.  A task could be all the functionality of a substem like a robot arm, 
         * or could be an individual action appy to a subsytem, such as up reading a gamepad and applying it to the drivcetrain.
         * Use 'public static' because these are single objects. 
         */

        public static TaskEnableRobot taskEnableRobot = new TaskEnableRobot();
        public static TaskDirectControlArm taskDirectControlArm = new TaskDirectControlArm();
        public static TaskDirectControlWheel taskDirectControlWheel = new TaskDirectControlWheel();
        public static TaskDisplay taskDisplay = new TaskDisplay();
        public static TaskOnceAbsoluteArmSensor taskOnceAbsoluteArmSensor = new TaskOnceAbsoluteArmSensor();
        public static TaskServoArmPos taskServoArmPos = new TaskServoArmPos();
        public static TaskServoWheelSpeed taskServoWheelSpeed = new TaskServoWheelSpeed();
        public static TaskLowBatteryDetect taskLowBatteryDetect = new TaskLowBatteryDetect();

        /* insert all Tasks below in the Full List so they get auto inserted, see Program.cs to see how this works.*/
        public static ILoopable[] FullList = {
            taskServoArmPos,
            taskServoWheelSpeed,
            taskDirectControlArm,
            taskDirectControlWheel,
            taskEnableRobot,
            taskDisplay,
            taskOnceAbsoluteArmSensor,
            taskLowBatteryDetect,
        };
    }
}
