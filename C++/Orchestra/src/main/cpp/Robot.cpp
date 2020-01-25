/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit() {
    _orchestra = new Orchestra();
    _joy = new frc::Joystick(0);
    for(int i = 0; i < INSTRUMENT_COUNT; ++i) {
        /* Instantiate an instrument */
        _instruments[i] = new TalonFX(i + 1);
        /* Add instruments to orchestra */
        _orchestra->AddInstrument(*_instruments[i]);
    }

    {
        _songs.push_back("song1.chrp");
        _songs.push_back("song2.chrp");
        _songs.push_back("song3.chrp");
        _songs.push_back("song4.chrp");
        _songs.push_back("song5.chrp");
        _songs.push_back("song6.chrp");
        _songs.push_back("song7.chrp");
        _songs.push_back("song8.chrp");
        _songs.push_back("song9.chrp");
        _songs.push_back("song10.chrp");
    }
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
    if(_joy->GetRawButton(1) && _lastButton != 1) {
      /* Play the song */
      _orchestra->Play();
      _lastButton = 1;

      std::cout << "Playing" << std::endl;
    }
    if(_joy->GetRawButton(2) && _lastButton != 2) {
      /* Pause */
      _orchestra->Pause();
      _lastButton = 2;

      std::cout << "Paused" << std::endl;
    }
    if(_joy->GetRawButton(3) && _lastButton != 3) {
      /* Stop */
      _orchestra->Stop();
      _lastButton = 3;

      std::cout << "Stopped" << std::endl;
    }
    if(_joy->GetRawButton(4) && _lastButton != 4) {
      /* Load selected song */
      _orchestra->LoadMusic(_songs[_songSelection]);
      _lastButton = 4;

      std::cout << "Loaded " << _songs[_songSelection] << std::endl;
    }
    if(_joy->GetRawButton(5)) {
        _orchestra->ClearInstruments();
    }

    int currentPOV = _joy->GetPOV();
    if(currentPOV == 90 && _lastPOV != 90) {
      _songSelection++;
      if(_songSelection >= (int)_songs.size()) _songSelection = 0;

      std::cout << "Song selected is: " << _songs[_songSelection] << std::endl;
    }
    if(currentPOV == 270 && _lastPOV != 270) {
      _songSelection--;
      if(_songSelection < 0) _songSelection = (int)_songs.size() - 1;
      
      std::cout << "Song selected is: " << _songs[_songSelection] << std::endl;
    }
    _lastPOV = currentPOV;
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
