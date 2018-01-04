/**
 * Example demonstrating the velocity closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 *
 * Be sure to select the correct feedback sensor using SetFeedbackDevice() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the SetSensorDirection() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target velocity.
 *
 * Tweak the PID gains accordingly.
 */
#include "WPILib.h"
#include "ctre/Phoenix.h"

class Robot: public IterativeRobot {
private:
	TalonSRX * _talon = new TalonSRX(3);
	Joystick * _joy = new Joystick(0);
	std::string _sb;
	int _loops = 0;

	void RobotInit() {
        /* first choose the sensor */
		_talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
		_talon->SetSensorPhase(true);

		/* set the peak and nominal outputs, 12V means full */
		_talon->ConfigNominalOutputForward(0, 10);
		_talon->ConfigNominalOutputReverse(0, 10);
		_talon->ConfigPeakOutputForward(1, 10);
		_talon->ConfigPeakOutputReverse(-1, 10);
		/* set closed loop gains in slot0 */
		_talon->Config_kF(0, 0.1097, 10);
		_talon->Config_kP(0, 0.22, 10);
		_talon->Config_kI(0, 0.0, 10);
		_talon->Config_kD(0, 0.0, 10);
	}
	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		/* get gamepad axis */
		double leftYstick = _joy->GetY();
		double motorOutput = _talon->GetMotorOutputVoltage() / _talon->GetBusVoltage();
		/* prepare line to print */
		_sb.append("\tout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tspd:");
		_sb.append(std::to_string(_talon->GetSelectedSensorVelocity(0)));
		/* while button1 is held down, closed-loop on target velocity */
		if (_joy->GetRawButton(1)) {
        	/* Speed mode */
			double targetSpeed = leftYstick * 1500.0; /* 1500 RPM in either direction */
        	_talon->Set(ControlMode::Velocity, targetSpeed); /* 1500 RPM in either direction */

			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(std::to_string(_talon->GetClosedLoopError(0)));
			_sb.append("\ttrg:");
			_sb.append(std::to_string(targetSpeed));
        } else {
			/* Percent voltage mode */
			_talon->Set(ControlMode::PercentOutput, leftYstick);
		}
		/* print every ten loops, printing too much too fast is generally bad for performance */
		if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n",_sb.c_str());
		}
		_sb.clear();
	}
};

START_ROBOT_CLASS(Robot)
