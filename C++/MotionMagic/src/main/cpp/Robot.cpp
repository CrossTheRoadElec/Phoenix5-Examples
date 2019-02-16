#include "Robot.h"
#include <sstream>

void Robot::RobotInit() {
    _talon = new TalonSRX(1);
    _joy = new frc::Joystick(0);

    /* Factory default hardware to prevent unexpected behavior */
    _talon->ConfigFactoryDefault();

    /* Configure Sensor Source for Pirmary PID */
    _talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
                                        0, 
                                        10);

    /**
     * Configure Talon SRX Output and Sesnor direction accordingly
     * Invert Motor to have green LEDs when driving Talon Forward / Requesting Postiive Output
     * Phase sensor to have positive increment when driving Talon Forward (Green LED)
     */
    _talon->SetSensorPhase(true);
    _talon->SetInverted(false);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    _talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    _talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
    _talon->ConfigNominalOutputForward(0, 10);
    _talon->ConfigNominalOutputReverse(0, 10);
    _talon->ConfigPeakOutputForward(1, 10);
    _talon->ConfigPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
    _talon->SelectProfileSlot(0, 0);
    _talon->Config_kF(0, 0.3, 10);
    _talon->Config_kP(0, 0.1, 10);
    _talon->Config_kI(0, 0.0, 10);
    _talon->Config_kD(0, 0.0, 10);

    /* Set acceleration and vcruise velocity - see documentation */
    _talon->ConfigMotionCruiseVelocity(1500, 10);
    _talon->ConfigMotionAcceleration(1500, 10);

    /* Zero the sensor */
    _talon->SetSelectedSensorPosition(0, 0, 10);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
    /* Get gamepad axis - forward stick is positive */
    double leftYstick = -1.0 * _joy->GetY();
    if (fabs(leftYstick) < 0.10) { leftYstick = 0;} /* deadband 10% */

    /* Get current Talon SRX motor output */
    double motorOutput = _talon->GetMotorOutputPercent();
    std::stringstream sb;
    /* Prepare line to print */
    sb << "\tOut%:" << motorOutput;
    sb << "\tVel:" << _talon->GetSelectedSensorVelocity(0);

    if(_joy->GetRawButton(2))
    {
        /* Zero the sensor */
        _talon->SetSelectedSensorPosition(0, 0, 10);
    }

    /**
     * Peform Motion Magic when Button 1 is held,
     * else run Percent Output, which can be used to confirm hardware setup.
     */
    if (_joy->GetRawButton(1)) {
        /* Motion Magic */ 
        
        /*4096 ticks/rev * 10 Rotations in either direction */
        double targetPos = leftYstick * 4096 * 10.0;
        _talon->Set(ControlMode::MotionMagic, targetPos);

        /* Append more signals to print when in speed mode */
        sb << "\terr:" << _talon->GetClosedLoopError(0);
        sb << "\ttrg:" << targetPos;
    } else {
        /* Percent Output */

        _talon->Set(ControlMode::PercentOutput, leftYstick);
    }

    if(_joy->GetRawButtonPressed(6))
    {
        /* Increase smoothing */
        ++_smoothing;
        if(_smoothing > 8) _smoothing = 8;
        std::cout << "Smoothing is set to: " << _smoothing << std::endl;
        _talon->ConfigMotionSCurveStrength(_smoothing, 0);
    }
    if(_joy->GetRawButtonPressed(5))
    {
        /* Decreasing smoothing */
        --_smoothing;
        if(_smoothing < 0) _smoothing = 0;
        std::cout << "Smoothing is set to: " << _smoothing << std::endl;
        _talon->ConfigMotionSCurveStrength(_smoothing, 0);
    }
    

    /* Instrumentation */
    Instrum::Process(_talon, &sb);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
