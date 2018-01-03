#include "WPILib.h"
#include "ctre/Phoenix.h"

class MecanumDefaultCode : public IterativeRobot
{
	TalonSRX *lf = new TalonSRX(0); /*left front */
	TalonSRX *lr = new TalonSRX(1);/*left rear */
	TalonSRX *rf = new TalonSRX(2); /*right front */
	TalonSRX *rr = new TalonSRX(3); /*right rear */
public:
	MecanumDrive *m_robotDrive;		// RobotDrive object using PWM 1-4 for drive motors
	Joystick *m_driveStick;			// Joystick object on USB port 1 (mecanum drive)public:
	AnalogGyro gyro;
	/**
	 * Constructor for this "MecanumDefaultCode" Class.
	 */
	MecanumDefaultCode(void) : gyro(0)
	{
		/* Set every Talon to reset the motor safety timeout. */
		lf->Set(ControlMode::PercentOutput, 0);
		lr->Set(ControlMode::PercentOutput, 0);
		rf->Set(ControlMode::PercentOutput, 0);
		rr->Set(ControlMode::PercentOutput, 0);

		//Invert Right Side
		rf->GetWPILIB_SpeedController().SetInverted(true);
		rr->GetWPILIB_SpeedController().SetInverted(true);

		// Create a RobotDrive object using PWMS 1, 2, 3, and 4
		m_robotDrive = new MecanumDrive(lf->GetWPILIB_SpeedController(),
				lr->GetWPILIB_SpeedController(), rf->GetWPILIB_SpeedController(),
				rr->GetWPILIB_SpeedController());
		m_robotDrive->SetExpiration(0.5);
		m_robotDrive->SetSafetyEnabled(false);
		// Define joystick being used at USB port #0 on the Drivers Station
		m_driveStick = new Joystick(0);
	}
	void TeleopInit()
	{
		gyro.Reset();
	}
	/** @return 10% deadband */
	double Db(double axisVal)
	{
		if(axisVal < -0.10)
			return axisVal;
		if(axisVal > +0.10)
			return axisVal;
		return 0;
	}
	/**
	 * Gets called once for each new packet from the DS.
	 */
	void TeleopPeriodic(void)
	{
		float angle = gyro.GetAngle();
		//std::cout << "Angle : " << angle << std::endl;
		m_robotDrive->DriveCartesian(			Db(m_driveStick->GetX()),
												Db(m_driveStick->GetY()),
												Db(m_driveStick->GetZ()),
												angle);
		/* my right side motors need to drive negative to move robot forward */

		/* on button 5, reset gyro angle to zero */
		if(m_driveStick->GetRawButton(5))
			gyro.Reset();
	}
};
START_ROBOT_CLASS(MecanumDefaultCode);
