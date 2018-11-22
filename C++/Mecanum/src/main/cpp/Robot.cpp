#include "frc/WPILib.h"
#include "frc/drive/MecanumDrive.h"
#include "ctre/Phoenix.h"

using namespace frc;

class MecanumDefaultCode: public TimedRobot {
	/* Phoenix Talons */
	WPI_TalonSRX *lf = new WPI_TalonSRX(0); /*left front */
	WPI_TalonSRX *lr = new WPI_TalonSRX(1);	/*left rear */
	WPI_TalonSRX *rf = new WPI_TalonSRX(2); /*right front */
	WPI_TalonSRX *rr = new WPI_TalonSRX(3); /*right rear */
public:
	MecanumDrive *m_robotDrive;	// RobotDrive object using PWM 1-4 for drive motors
	Joystick *m_driveStick;	// Joystick object on USB port 1 (mecanum drive)public:
	AnalogGyro gyro;
	/**
	 * Constructor for this "MecanumDefaultCode" Class.
	 */
	MecanumDefaultCode(void) :
			gyro(0) {
		/* Set every Talon to reset the motor safety timeout. */
		lf->Set(0);
		lr->Set(0);
		rf->Set(0);
		rr->Set(0);

		//Invert Right Side
		rf->SetInverted(true);
		rr->SetInverted(true);

		// Create a RobotDrive object using PWMS 1, 2, 3, and 4
		m_robotDrive = new MecanumDrive(*lf, *lr, *rf, *rr);
		m_robotDrive->SetExpiration(0.5);
		m_robotDrive->SetSafetyEnabled(false);
		// Define joystick being used at USB port #0 on the Drivers Station
		m_driveStick = new Joystick(0);
	}
	void TeleopInit() {
		gyro.Reset();
	}
	/** @return 10% deadband */
	double Db(double axisVal) {
		if (axisVal < -0.10)
			return axisVal;
		if (axisVal > +0.10)
			return axisVal;
		return 0;
	}
	/**
	 * Gets called once for each new packet from the DS.
	 */
	void TeleopPeriodic(void) {
		float angle = gyro.GetAngle();
		//std::cout << "Angle : " << angle << std::endl;
		m_robotDrive->DriveCartesian(			Db(m_driveStick->GetX()),
												Db(m_driveStick->GetY()),
												Db(m_driveStick->GetZ()),
												angle);
		/* my right side motors need to drive negative to move robot forward */

		/* on button 5, reset gyro angle to zero */
		if (m_driveStick->GetRawButton(5))
			gyro.Reset();
	}
};
START_ROBOT_CLASS(MecanumDefaultCode);
