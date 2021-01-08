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
 * Helpful examle explaining WPI's motor safety feature.
 * The goal is to auto-neutral your motors if your application stops calling set().
 *
 * Motor safety features can be a trap for teams where:
 *  - They unknowingly rely on this because they don't explicitly turn off motors at end of auton.
 *      Then when they disable it to workaround a problem, auton robot drives into a wall.
 *  - This can potentioally cause erroneuos motor disables due to intermittent
 *       lags in loop time (historically caused by excessive printing, logging, or competition WiFi lag).
 * 
 * Some tips:
 * - Always have a team member ready to disable the robot via Driver Station!
 * - Neutral your motor outputs explicitly in your disabled loop.
 * - Ensure you set motor output to zero when actions are complete.
 * - Use motor safety in contexts that makes sense, i.e. during source-level breakpoint/debugging.
 * 
 * Controls:
 * Left Joystick Y-Axis: Drive robot in forward and reverse direction
 * Right Joystick X-Axis: Turn robot in right and left direction
 * Button 1: Stops updating the drive train.  If safeties are on, this should neutral the motor and trigger
 *      a Driver Station message. If safeties are off MOTORS WILL CONTINUE LAST COMMAND.
 * 
 */
#include "Robot.h"
#include "ctre/Phoenix.h"
#include "frc/PWMTalonSRX.h"
#include "frc/drive/DifferentialDrive.h"
#include "frc/Joystick.h"
#include "PhysicsSim.h"

/* Master Talons for arcade drive */
WPI_TalonSRX _left(1);
WPI_TalonSRX _rght(0);

/* Construct drivetrain by providing master motor controllers */
frc::DifferentialDrive *_drive;
frc::Joystick * _joy;  /* Joystick for control */

int _loops = 0; // slow print to the DS

void Robot::SimulationInit() {
    PhysicsSim::GetInstance().AddTalonSRX(_left, 0.75, 4000);
    PhysicsSim::GetInstance().AddTalonSRX(_rght, 0.75, 4000);
}
void Robot::SimulationPeriodic() {
    PhysicsSim::GetInstance().Run();
}

void Robot::RobotInit()
{
    _drive = new frc::DifferentialDrive(_left, _rght);
    _joy =  new frc::Joystick(0); 
    /* Factory Default all hardware to prevent unexpected behaviour */
    _left.ConfigFactoryDefault();
    _rght.ConfigFactoryDefault();
}


void Robot::TeleopInit()
{
    /* Motor controllers default motor safety OFF.
        WPI drive trains default motor safety ON.
        Experiment with different enables below.... */
    //_left.SetSafetyEnabled(true);
    //_rght.SetSafetyEnabled(true);
    //_drive->SetSafetyEnabled(false); 

    /* Factory Default all hardware to prevent unexpected behaviour */
    _left.ConfigFactoryDefault();
    _rght.ConfigFactoryDefault();

    /**
     * Drive robot forward and make sure all motors spin the correct way. Toggle
     * booleans accordingly....
     */
    _left.SetInverted(false); // <<<<<< Adjust this until robot drives forward when stick is forward
    _rght.SetInverted(true);  // <<<<<< Adjust this until robot drives forward when stick is forward

    /*
     * diff drive assumes (by default) that right side must be negative to move
     * forward. Change to 'false' so positive/green-LEDs moves robot forward
     */
    _drive->SetRightSideInverted(false); // do not change this
}
void Robot::TeleopPeriodic()
{
    /* Gamepad processing */
    double forward = -1.0 * _joy->GetY(); // Sign this so forward is positive
    double turn = +1.0 * _joy->GetZ();    // Sign this so right is positive

    /**
     * Print the joystick values to sign them, comment out this line after checking
     * the joystick directions.
     */
    if (++_loops >= 10)
    { /* slow print to the DS */
        _loops = 0;
        printf("JoyY:%f  turn:%f \n", forward, turn);
    }

    if (_joy->GetRawButton(1))
    {
        /* if button 0 is pressed, stop calling arcadeDrive, which calls set() */
    }
    else
    {
        /* button is not held, update the drivetrain like normal */
        _drive->ArcadeDrive(forward, turn);
    }
}


void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
