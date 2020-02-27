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
 */

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <string>
#include "ctre/Phoenix.h"

#define TALON_COUNT 2
#define SONG_COUNT 11

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
private:

  int GetButton();
  void LoadMusicSelection(int offset);


    /* The orchestra object that holds all the instruments */
    Orchestra *_orchestra;

    /* Talon FXs to play music through.  
    More complex music MIDIs will contain several tracks, requiring multiple instruments.  */
    TalonFX **_fxes;

    /* An array of songs that are available to be played, can you guess the song/artists? */
    std::string _songs[SONG_COUNT] = {
      "song1.chrp",
      "song2.chrp",
      "song3.chrp",
      "song4.chrp",
      "song5.chrp",
      "song6.chrp",
      "song7.chrp",
      "song8.chrp",
      "song9.chrp", /* the remaining songs play better with three or more FXs */
      "song10.chrp",
      "song11.chrp",
    };

    /* track which song is selected for play */
    int _songSelection = 0;

    /* overlapped actions */
    int _timeToPlayLoops = 0;

    /* joystick vars */
    frc::Joystick *_joy;
    int _lastButton = 0;
    int _lastPOV = 0;
};
