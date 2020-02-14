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
#pragma once

#include "ctre/Phoenix.h"

class FurElise {
  int _noteIter = 0; // !< Iterates over the notes
  int _timeoutMs = 0; // !< Count down from prev notes to track when to play next note.
  int _state = 0; // !< State machine to track tone vs off state.
  double _frequencyToPlay = 0; // !< The actual frequency to play through the TalonFX.

    /* note defs */
  #define NOTE_OFF 0
  #define NOTE_E6 1318.51
  #define NOTE_B5 987.767
  #define NOTE_A5sharp 932.328
  #define NOTE_A5 880
  #define NOTE_G5sharp 830.609
  #define NOTE_G5 783.991
  #define NOTE_F5sharp 739.989
  #define NOTE_F5 698.456
  #define NOTE_E5 659.255
  #define NOTE_D5sharp 622.254
  #define NOTE_DS5 622.254
  #define NOTE_D5 587.33
  #define NOTE_C5sharp 554.365
  #define NOTE_C5 523.251
  #define NOTE_B4 493.883
  #define NOTE_A4sharp 466.164
  #define NOTE_A4 440
  #define NOTE_G4sharp 415.305
  #define NOTE_GS4 415.305
  #define NOTE_G4 391.995
  #define NOTE_F4sharp 369.994
  #define NOTE_F4 349.228
  #define NOTE_E4 329.628
  #define NOTE_D4sharp 311.127
  #define NOTE_DS4 311.127
  #define NOTE_D4 293.665
  #define NOTE_C4sharp 277.183
  #define NOTE_C4 261.626

  #define NOTE_COUNT 96

  // Fur Elise: https://en.wikipedia.org/wiki/F%C3%BCr_Elise
  double notes[NOTE_COUNT] {  NOTE_E5,  NOTE_DS5, NOTE_E5,   NOTE_DS5, NOTE_E5,  NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4,   NOTE_C4,
                    NOTE_E4,  NOTE_A4,  NOTE_B4,  NOTE_E4,   NOTE_GS4, NOTE_B4,  NOTE_C5, NOTE_E4, NOTE_E5, NOTE_DS5,  NOTE_E5, NOTE_DS5,
                    NOTE_E5,  NOTE_B4,  NOTE_D5,  NOTE_C5,   NOTE_A4,  NOTE_C4,  NOTE_E4, NOTE_A4, NOTE_B4, NOTE_E4,   NOTE_C5, NOTE_B4,
                    NOTE_A4,  NOTE_B4,  NOTE_C5,  NOTE_D5,   NOTE_E5,  NOTE_G4,  NOTE_F5, NOTE_E5, NOTE_D5, NOTE_F4,   NOTE_E5, NOTE_D5,
                    NOTE_C5,  NOTE_E4,  NOTE_D5,  NOTE_C5,   NOTE_B4,  NOTE_E4,  NOTE_E5, NOTE_E4, NOTE_E5, NOTE_E4,   NOTE_E5, NOTE_E4,
                    NOTE_E5,  NOTE_DS4, NOTE_E5,  NOTE_D4,   NOTE_E5,  NOTE_DS4, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5,   NOTE_A4, NOTE_C4,
                    NOTE_E4,  NOTE_A4,  NOTE_B4,  NOTE_E4,   NOTE_GS4, NOTE_B4,  NOTE_C5, NOTE_E4, NOTE_E5, NOTE_DS5,  NOTE_E5, NOTE_DS5,
                    NOTE_E5,  NOTE_B4,  NOTE_D5,  NOTE_C5,   NOTE_A4,  NOTE_C4,  NOTE_E4, NOTE_A4, NOTE_B4, NOTE_E4,   NOTE_C5, NOTE_B4,
                    NOTE_A4,  NOTE_OFF };

  int durationsMs[NOTE_COUNT] { 111, 111, 111, 111, 111, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111,
      111, 111, 111, 111, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111,
      333, 111, 111, 111, 333, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111,
      111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111, 111, 111, 111, 111, 111, 111, 333, 111,
      111, 111, 333, 125, 125, 125, 999, 111 };

public:

  double GetMusicFrequency(int dt_ms) {
    if (_timeoutMs >= 0) {
      /* still waiting for note duration */
      _timeoutMs -= dt_ms;
    } else if (_noteIter >= NOTE_COUNT) {
      /* all notes played, start all over */
      _noteIter = 0;
    } else {
      /* get the next note and duration */
      int freq = (int) notes[_noteIter];
      int durMs = (int) durationsMs[_noteIter];

      /* state machine */
      switch (_state) {
      case 0:
        /* save the frequency to play */
        _frequencyToPlay = freq;
        /* arm the timeout, jump to state 1 */
        _timeoutMs = durMs;
        _state = 1;
        break;

      case 1: // play off for a bit. You don't necessarily need this, depends on the type of
              // music
        /* release the piano key for a bit */
        _frequencyToPlay = 0;
        /* back to state 0 for next press */
        _state = 0;
        _timeoutMs = 20;
        ++_noteIter;
        break;
      }
    }
    /* give caller the frequency */
    return _frequencyToPlay;
  }
};
