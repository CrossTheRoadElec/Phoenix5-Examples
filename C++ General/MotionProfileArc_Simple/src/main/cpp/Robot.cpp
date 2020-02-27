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
#include "MotionProfile.h"
#include "Instrum.h"

void Robot::RobotInit() 
{
    /* Construct global variables */
    _rightMaster = new TalonSRX(1);
    _leftMaster = new TalonSRX(2);
    _pidgey = new PigeonIMU(3); //This uses a CAN pigeon, as opposed to a gadgeteer pigeon
    _joystick = new frc::Joystick(0);
    _bufferedStream = new BufferedTrajectoryPointStream();

    _plotThread = new PlotThread(_rightMaster);

    /* Initialize buffer with motion profile */
    InitBuffer(kMotionProfile, kMotionProfileSz, 0.25); //Do a quarter (0.25) rotation to the left
    _state = 0;


    _masterConfig = new MasterProfileConfiguration(_leftMaster, _pidgey);
    _followConfig = new FollowerProfileConfiguration();

    _rightMaster->ConfigAllSettings(*_masterConfig);
    _leftMaster->ConfigAllSettings(*_followConfig);

    _rightMaster->SetSensorPhase(true);
    _leftMaster->SetSensorPhase(false);

    _rightMaster->SetInverted(true);
    _leftMaster->SetInverted(false);

    _rightMaster->SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 20); //Telemetry using Phoenix Tuner
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() 
{
    /* get joystick button and stick */
    bool bPrintValues = _joystick->GetRawButton(2);
    bool bFireMp = _joystick->GetRawButton(1);
    double axis = -_joystick->GetRawAxis(1);
    double turn = _joystick->GetRawAxis(2);

    /* if button is up, just drive the motor in PercentOutput */
    if (bFireMp == false) {
        _state = 0;
    }

    switch (_state) {
        /* drive master talon normally */
        case 0:
            _rightMaster->Set(ControlMode::PercentOutput, axis, DemandType_ArbitraryFeedForward, -turn);
            _leftMaster->Set(ControlMode::PercentOutput, axis, DemandType_ArbitraryFeedForward, turn);
            if (bFireMp == true) {
                /* go to MP logic */
                _state = 1;
            }
            break;

        /* fire the MP, and stop calling set() since that will cancel the MP */
        case 1:
            _rightMaster->GetSensorCollection().SetQuadraturePosition(0);
            _leftMaster->GetSensorCollection().SetQuadraturePosition(0);
            _pidgey->SetYaw(0);
            /* wait for 10 points to buffer in firmware, then transition to MP */
            _leftMaster->Follow(*_rightMaster, FollowerType_AuxOutput1);
            _rightMaster->StartMotionProfile(*_bufferedStream, 10, ControlMode::MotionProfileArc);
            _state = 2;
            Instrum::PrintLine("MP started");
            break;

        /* wait for MP to finish */
        case 2:
            if (_rightMaster->IsMotionProfileFinished()) {
                Instrum::PrintLine("MP finished");
                _state = 3;
            }
            break;

        /* MP is finished, nothing to do */
        case 3:
            break;
    }

    /* print MP values */
    Instrum::Loop(bPrintValues, _rightMaster);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::InitBuffer(const double profile[][3], int totalCnt, double rotations)
{
    bool forward = true; // set to false to drive in opposite direction of profile (not really needed
                         // since you can use negative numbers in profile).

    TrajectoryPoint point; // temp for for loop, since unused params are initialized
                           // automatically, you can alloc just one

    /* clear the buffer, in case it was used elsewhere */
    _bufferedStream->Clear();

    double turnAmount = rotations * 8192.0; //8192 units per rotation for a pigeon


    /* Insert every point into buffer, no limit on size */
    for (int i = 0; i < totalCnt; ++i) {

        double direction = forward ? +1 : -1;
        double positionRot = profile[i][0];
        double velocityRPM = profile[i][1];
        int durationMilliseconds = (int) profile[i][2];

        /* for each point, fill our structure and pass it to API */
        point.timeDur = durationMilliseconds;
        point.position = direction * positionRot * 4096; // Convert Revolutions to
                                                         // Units
        point.velocity = direction * velocityRPM * 4096 / 600.0; // Convert RPM to
                                                                 // Units/100ms
        
        /** 
         * Here is where you specify the heading of the robot at each point. 
         * In this example we're linearly interpolating creating a segment of a circle to follow
         */
        point.auxiliaryPos = turnAmount * ((double)i / (double)totalCnt); //Linearly interpolate the turn amount to do a circle
        point.auxiliaryVel = 0;


        point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
        point.profileSlotSelect1 = 1; /* which set of gains would you like to use [0,3]? */
        point.zeroPos = (i == 0); /* set this to true on the first point */
        point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
        point.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

        point.useAuxPID = true; /* Using auxiliary PID */
        _bufferedStream->Write(point);
    }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
