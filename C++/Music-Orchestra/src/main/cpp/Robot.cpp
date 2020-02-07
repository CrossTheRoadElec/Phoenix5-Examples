/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products.
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
 * Example plays music sourced from a MIDI file through several Talon FXs.
 * Use Tuner to convert a MIDI file into a CHRP (chirp) file. 
 * These are then placed into the deploy folder of this project so that they are copied into the robot controller (on deploy).
 * 
 * Supported Version:
 * 	- Talon FX: 20.2.3.0 or newer
 *
 * Feature Video: https://youtu.be/MTGScSS_iaQ
 */

#include "Robot.h"


int Robot::GetButton() {
    for (int i = 1; i < 9; ++i) {
        if (_joy->GetRawButton(i)) {
            return i;
        }
    }
    return 0;
}

void Robot::LoadMusicSelection(int offset)
{
    /* increment song selection */
    _songSelection += offset;
    /* wrap song index in case it exceeds boundary */
    if (_songSelection >= SONG_COUNT) {
        _songSelection = 0;
    }
    if (_songSelection < 0) {
        _songSelection = SONG_COUNT - 1;
    }
    /* load the chirp file */
    _orchestra->LoadMusic(_songs[_songSelection]); 

    /* print to console */
    printf("Song selected is: %s.  Press left/right on d-pad to change.\n", _songs[_songSelection].c_str());
    
    /* schedule a play request, after a delay.  
        This gives the Orchestra service time to parse chirp file.
        If play() is called immedietely after, you may get an invalid action error code. */
    _timeToPlayLoops = 10;
}

void Robot::RobotInit() {
    /* Create the orchestra with the TalonFX instruments */
    _orchestra = new Orchestra();

    _fxes = new TalonFX * [TALON_COUNT];
    /* Initialize the TalonFX's to be used */
    for (int i = 0; i < TALON_COUNT; ++i) {
        _fxes[i] = new TalonFX(i+1);
        _orchestra->AddInstrument(*_fxes[i]);
    }
    _joy = new frc::Joystick(0);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    /* load whatever file is selected */
    LoadMusicSelection(0);
}
void Robot::TeleopPeriodic() {
    /* poll gamepad */
    int btn = GetButton();
    int currentPOV = _joy->GetPOV();

    /* if song selection changed, auto-play it */
    if (_timeToPlayLoops > 0) {
        --_timeToPlayLoops;
        if (_timeToPlayLoops == 0) {
            /* scheduled play request */
            printf("Auto-playing song.\n");
            _orchestra->Play();
        }
    }


    /* has a button been pressed? */
    if (_lastButton != btn) {
        _lastButton = btn;

        switch (btn) {
            case 1: /* toggle play and paused */
                if (_orchestra->IsPlaying()) {
                    _orchestra->Pause();
                    printf("Song paused\n");
                }  else {
                    _orchestra->Play();
                    printf("Playing song...\n");
                }
                break;
                
            case 2: /* toggle play and stop */
                if (_orchestra->IsPlaying()) {
                    _orchestra->Stop();
                    printf("Song stopped.\n");
                }  else {
                    _orchestra->Play();
                    printf("Playing song...\n");
                }
                break;
        }
    }

    /* has POV/D-pad changed? */
    if (_lastPOV != currentPOV) {
        _lastPOV = currentPOV;

        switch (currentPOV) {
            case 90:
                /* increment song selection */
                LoadMusicSelection(+1);
                break;
            case 270:
                /* decrement song selection */
                LoadMusicSelection(-1);
                break;
        }
    }
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
