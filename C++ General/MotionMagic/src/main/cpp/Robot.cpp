/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The MotionMagic example demonstrates the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually. This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction. If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Ensure your feedback device is in-phase with the motor,
 * and you have followed the walk-through in the Talon SRX Software Reference Manual.
 * 
 * Controls:
 * Button 1: When held, put Talon in Motion Magic mode and allow Talon to drive [-10, 10] 
 * 	rotations.
 * Button 2: When pushed, the selected feedback sensor gets zero'd
 * Button 5(Left shoulder): When pushed, will decrement the smoothing of the motion magic down to 0
 * Button 6(Right shoulder): When pushed, will increment the smoothing of the motion magic up to 8
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon SRX forward and reverse, use to confirm hardware setup.
 * Right Joystick Y-Axis:
 * 	+ Motion Maigic: Servo Talon SRX forward and reverse, [-10, 10] rotations.
 * 
 * Gains for Motion Magic
 * 
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */
#include "Robot.h"
#include <sstream>

#include "PhysicsSim.h"

void Robot::SimulationInit() {
    PhysicsSim::GetInstance().AddTalonSRX(*_talon, 0.75, 3400, false);
}
void Robot::SimulationPeriodic() {
    PhysicsSim::GetInstance().Run();
}

void Robot::RobotInit() {
    _talon = new WPI_TalonSRX(1);
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
    _talon->SetSensorPhase(false);
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
    double RightYstick = -1.0 * _joy->GetRawAxis(5);
    if (fabs(leftYstick) < 0.10) { leftYstick = 0;} /* deadband 10% */
    if (fabs(RightYstick) < 0.10) { RightYstick = 0;} /* deadband 10% */

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
        double targetPos = RightYstick * 4096 * 10.0;
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
