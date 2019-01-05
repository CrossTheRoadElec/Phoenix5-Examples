#include "Tasks/TaskMainLoop.h"
#include "Platform/Platform.h"

/* Destructor */
TaskMainLoop::~TaskMainLoop() { }

/* ILoopable */
void TaskMainLoop::OnStart() {
	/* default to LED strip animation */
	Schedulers::PeriodicTasks->Start(Tasks::taskAnimateLEDStrip);
	Schedulers::PeriodicTasks->Stop(Tasks::taskDirectControlArm);
	Schedulers::PeriodicTasks->Stop(Tasks::taskLIDARControlLEDStrip);
}
void TaskMainLoop::OnStop() {
}
bool TaskMainLoop::IsDone() {
	return false;
}
void TaskMainLoop::OnLoop() {
	bool controllerState = false;

	/* No method of determining if controller is connected */
	if (controllerState == false) {
		Schedulers::PeriodicTasks->Stop(Tasks::taskPWMmotorController);
	} else {
		/* Just for testing, can be enabled by user by changing controllerState to true */
		Schedulers::PeriodicTasks->Start(Tasks::taskPWMmotorController);
	}

	if (Hardware::gamepad->GetRawButton(6)) {
		/* no gamepad present OR user is holding R shoulder btn. Just roll thru color wheel*/
		Schedulers::PeriodicTasks->Start(Tasks::taskAnimateLEDStrip);
		Schedulers::PeriodicTasks->Stop(Tasks::taskDirectControlArm);
		Schedulers::PeriodicTasks->Stop(Tasks::taskLIDARControlLEDStrip);
	} else if (Hardware::gamepad->GetRawButton(5)) {
		/* let user control LED with sticks */
		Schedulers::PeriodicTasks->Stop(Tasks::taskAnimateLEDStrip);
		Schedulers::PeriodicTasks->Start(Tasks::taskDirectControlArm);
		Schedulers::PeriodicTasks->Stop(Tasks::taskLIDARControlLEDStrip);

		Schedulers::PeriodicTasks->Start(Tasks::taskMeasurePulseSensors);

	} else if (Hardware::gamepad->GetRawButton(7)) {
		Schedulers::PeriodicTasks->Stop(Tasks::taskAnimateLEDStrip);
		Schedulers::PeriodicTasks->Stop(Tasks::taskDirectControlArm);
		Schedulers::PeriodicTasks->Start(Tasks::taskLIDARControlLEDStrip);

		Schedulers::PeriodicTasks->Start(Tasks::taskMeasurePulseSensors);

	}
}

