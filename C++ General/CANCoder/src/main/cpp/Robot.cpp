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

#include "Robot.h"

#define PRINTOUT_DELAY 100

void PrintFaults(CANCoderFaults &faults)
{
    printf("Hardware fault: %s\t    Under Voltage fault: %s\t    Reset During Enable fault: %s\t    API Error fault: %s\n",
           faults.HardwareFault ? "True " : "False",
           faults.UnderVoltage ? "True " : "False",
           faults.ResetDuringEn ? "True " : "False",
           faults.APIError ? "True " : "False");
}
void PrintFaults(CANCoderStickyFaults &faults)
{
    printf("Hardware fault: %s\t    Under Voltage fault: %s\t    Reset During Enable fault: %s\t     API Error fault: %s\n",
           faults.HardwareFault ? "True " : "False",
           faults.UnderVoltage ? "True " : "False",
           faults.ResetDuringEn ? "True " : "False",
           faults.APIError ? "True " : "False");
}
void PrintValue(double val, std::string units, double timestamp)
{
    printf("%20f %-20s @ %f\n", val, units.c_str(), timestamp);
}
void PrintValue(MagnetFieldStrength val, std::string units, double timestamp)
{
    printf("%20d %-20s @ %f\n", val, units.c_str(), timestamp);
}

void Robot::RobotInit()
{
    _CANCoder = new CANCoder(0);
    _joy = new frc::Joystick(0);
}

int count = 0;
void Robot::RobotPeriodic()
{
    if (count++ >= PRINTOUT_DELAY / 10)
    {
        count = 0;

        /* Report position, absolute position, velocity, battery voltage */
        double posValue = _CANCoder->GetPosition();
        std::string posUnits = _CANCoder->GetLastUnitString();
        double posTstmp = _CANCoder->GetLastTimestamp();

        double absValue = _CANCoder->GetAbsolutePosition();
        std::string absUnits = _CANCoder->GetLastUnitString();
        double absTstmp = _CANCoder->GetLastTimestamp();

        double velValue = _CANCoder->GetVelocity();
        std::string velUnits = _CANCoder->GetLastUnitString();
        double velTstmp = _CANCoder->GetLastTimestamp();

        double batValue = _CANCoder->GetBusVoltage();
        std::string batUnits = _CANCoder->GetLastUnitString();
        double batTstmp = _CANCoder->GetLastTimestamp();

        /* Report miscellaneous attributes about the CANCoder */
        MagnetFieldStrength magnetStrength = _CANCoder->GetMagnetFieldStrength();
        std::string magnetStrengthUnits = _CANCoder->GetLastUnitString();
        double magnetStrengthTstmp = _CANCoder->GetLastTimestamp();

        printf("Position: ");
        PrintValue(posValue, posUnits, posTstmp);
        printf("Abs Pos : ");
        PrintValue(absValue, absUnits, absTstmp);
        printf("Velocity: ");
        PrintValue(velValue, velUnits, velTstmp);
        printf("Battery : ");
        PrintValue(batValue, batUnits, batTstmp);
        printf("Strength: ");
        PrintValue(magnetStrength, magnetStrengthUnits, magnetStrengthTstmp);

        /* Fault reporting */
        CANCoderFaults faults = {0};
        _CANCoder->GetFaults(faults);
        CANCoderStickyFaults stickyFaults = {0};
        _CANCoder->GetStickyFaults(stickyFaults);

        printf("Faults:\n");
        PrintFaults(faults);
        printf("Sticky Faults:\n");
        PrintFaults(stickyFaults);

        printf("\n\n");
    }
    if (_joy->GetRawButton(1))
    {
        _CANCoder->ClearStickyFaults();
    }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
