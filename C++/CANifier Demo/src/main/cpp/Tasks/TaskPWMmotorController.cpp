#include "Tasks/TaskPWMmotorController.h"
#include "ctre/Phoenix.h"
#include "Platform/Platform.h"

/* Destructor */
TaskPWMmotorController::~TaskPWMmotorController() { }

/* ILoopable */
void TaskPWMmotorController::OnStart() {
	/* If already running, do nothing */
	if (_running)
		return;

	/* Start transmitting neutral signal */
	_percentOutput = 0;
	Hardware::canifier->SetPWMOutput(Constants::MotorControllerCh, 0);
	Hardware::canifier->EnablePWMOutput(Constants::MotorControllerCh, true);

	/* Task is now running */
	_running = true;
}
void TaskPWMmotorController::OnStop() {
	/* Stop transmitting PWM */
	Hardware::canifier->EnablePWMOutput(Constants::MotorControllerCh, false);

	/* Task has stopped, now take note */
	_running = false;
}
bool TaskPWMmotorController::IsDone() {
	return false;
}
void TaskPWMmotorController::OnLoop() {
	/* Grab the axis and direct control the components */
	float axis = Hardware::gamepad->GetRawAxis(Constants::Gamepad_y);
	/* Scale to typical PWM widths */
	float pulseUs = LinearInterpolation::Calculate(axis, -1, 1000, +1, 2000); /* [-1,+1] => [1000,2000]us */
	/* Scale to period */
	float periodUs = 4200; // hard-coded for now, this will be settable in future firmware update.
	_percentOutput = pulseUs / periodUs;
	/* Set it */
	Hardware::canifier->SetPWMOutput(Constants::MotorControllerCh, _percentOutput);
}
