/**
 * Example demonstrating the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 *
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the reverseSensor() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target position.
 *
 * Tweak the PID gains accordingly.
 */
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "Constants.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	TalonSRX * _talon = new TalonSRX(0);
	Joystick * _joy = new Joystick(0);
	std::string _sb;
	int _loops = 0;
	bool _lastButton1 = false;
	/** save the target position to servo to */
	double targetPositionRotations;

	void RobotInit() {
		/* lets grab the 360 degree position of the MagEncoder's absolute position */
		int absolutePosition = _talon->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		/* use the low level API to set the quad encoder signal */
		_talon->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx,
				kTimeoutMs);

		/* choose the sensor and sensor direction */
		_talon->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);
		_talon->SetSensorPhase(true);

		/* set the peak and nominal outputs, 12V means full */
		_talon->ConfigNominalOutputForward(0, kTimeoutMs);
		_talon->ConfigNominalOutputReverse(0, kTimeoutMs);
		_talon->ConfigPeakOutputForward(1, kTimeoutMs);
		_talon->ConfigPeakOutputReverse(-1, kTimeoutMs);

		/* set closed loop gains in slot0 */
		_talon->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		_talon->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		_talon->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);
	}

	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		/* get gamepad axis */
		double leftYstick = _joy->GetY();
		double motorOutput = _talon->GetMotorOutputPercent();
		bool button1 = _joy->GetRawButton(1);
		bool button2 = _joy->GetRawButton(2);
		/* prepare line to print */
		_sb.append("\tout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tpos:");
		_sb.append(std::to_string(_talon->GetSelectedSensorPosition(kPIDLoopIdx)));
		/* on button1 press enter closed-loop mode on target position */
		if (!_lastButton1 && button1) {
			/* Position mode - button just pressed */
			targetPositionRotations = leftYstick * 10.0 * 4096; /* 10 Rotations in either direction */
			_talon->Set(ControlMode::Position, targetPositionRotations); /* 10 rotations in either direction */
		}
		/* on button2 just straight drive */
		if (button2) {
			/* Percent voltage mode */
			_talon->Set(ControlMode::PercentOutput, leftYstick);
		}
		/* if Talon is in position closed-loop, print some more info */
		if (_talon->GetControlMode() == ControlMode::Position) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(std::to_string(_talon->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(std::to_string(targetPositionRotations));
		}
		/* print every ten loops, printing too much too fast is generally bad for performance */
		if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n", _sb.c_str());
		}
		_sb.clear();
		/* save button state for on press detect */
		_lastButton1 = button1;
	}
};

START_ROBOT_CLASS(Robot)
