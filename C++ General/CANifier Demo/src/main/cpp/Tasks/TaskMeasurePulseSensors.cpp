#include "Tasks/TaskMeasurePulseSensors.h"
#include "ctre/Phoenix.h"
#if defined (_WIN32) || defined (_WIN64)
#else
#include "FRC_NetworkCommunication/CANSessionMux.h"
#endif
#include "Platform/Platform.h"

float TaskMeasurePulseSensors::GetMeasuredPulseWidthsUs(
		CANifier::PWMChannel pwmCh) {
	return _dutyCycleAndPeriods[(int) pwmCh][0];
}
/* Destructor */
TaskMeasurePulseSensors::~TaskMeasurePulseSensors() { }

/* ILoopable */
void TaskMeasurePulseSensors::OnStart() {
}
void TaskMeasurePulseSensors::OnStop() {
}
bool TaskMeasurePulseSensors::IsDone() {
	return false;
}
void TaskMeasurePulseSensors::OnLoop() {
	/* Retrieve PWM from the CANifier connected to our PWM source */
	Hardware::canifier->GetPWMInput(CANifier::PWMChannel::PWMChannel0,
			_dutyCycleAndPeriods[0]);
	Hardware::canifier->GetPWMInput(CANifier::PWMChannel::PWMChannel1,
			_dutyCycleAndPeriods[1]);
	Hardware::canifier->GetPWMInput(CANifier::PWMChannel::PWMChannel2,
			_dutyCycleAndPeriods[2]);
	Hardware::canifier->GetPWMInput(CANifier::PWMChannel::PWMChannel3,
			_dutyCycleAndPeriods[3]);

	/* Send CAN data */
	uint8_t data = 0;
	int status = 0;
	data = (uint64_t) (_dutyCycleAndPeriods[3][0] * 1000);
#if defined (_WIN32) || defined (_WIN64)
#else
	FRC_NetworkCommunication_CANSessionMux_sendMessage(0x1E040000, &data, 4, 0,
			&status);
#endif

}
