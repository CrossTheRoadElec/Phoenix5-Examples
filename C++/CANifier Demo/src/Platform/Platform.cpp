#include <Platform/Platform.h>

/* Hardware */
CTRE::CANifier* Hardware::canifier = new CTRE::CANifier(0);
frc::Joystick* Hardware::gamepad = new frc::Joystick(0);

/* Schedulers */
CTRE::Tasking::Schedulers::ConcurrentScheduler* Schedulers::PeriodicTasks = new CTRE::Tasking::Schedulers::ConcurrentScheduler();

/* Tasks */
TaskAnimateLEDStrip* Tasks::taskAnimateLEDStrip = new TaskAnimateLEDStrip();
TaskDirectControlLEDStrip* Tasks::taskDirectControlArm = new TaskDirectControlLEDStrip();
TaskPWMmotorController* Tasks::taskPWMmotorController = new TaskPWMmotorController();
TaskMeasurePulseSensors* Tasks::taskMeasurePulseSensors = new TaskMeasurePulseSensors();
TaskLIDARControlLEDStrip* Tasks::taskLIDARControlLEDStrip = new TaskLIDARControlLEDStrip();
TaskHSV* Tasks::taskHSVControlLedStrip = new TaskHSV();
TaskMainLoop* Tasks::taskMainLoop = new TaskMainLoop();

/* TaskList */
std::vector<CTRE::Tasking::ILoopable*> Tasks::FullList = {
		taskAnimateLEDStrip,
		taskDirectControlArm,
		taskPWMmotorController,
		taskMeasurePulseSensors,
		taskLIDARControlLEDStrip,
		taskHSVControlLedStrip,
		taskMainLoop,
};



