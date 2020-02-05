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
package frc.robot;

public class FurElise {

  int _noteIter = 0; // !< Iterates over the notes
  int _timeoutMs = 0; // !< Count down from prev notes to track when to play next note.
  int _state = 0; // !< State machine to track tone vs off state.
  double _frequencyToPlay = 0; // !< The actual frequency to play through the TalonFX.

  /* note defs */
  final double NOTE_OFF = (double) 0;
  final double NOTE_E6 = (double) 1318.51;
  final double NOTE_B5 = (double) 987.767;
  final double NOTE_A5sharp = (double) 932.328;
  final double NOTE_A5 = (double) 880;
  final double NOTE_G5sharp = (double) 830.609;
  final double NOTE_G5 = (double) 783.991;
  final double NOTE_F5sharp = (double) 739.989;
  final double NOTE_F5 = (double) 698.456;
  final double NOTE_E5 = (double) 659.255;
  final double NOTE_D5sharp = (double) 622.254;
  final double NOTE_DS5 = (double) 622.254;
  final double NOTE_D5 = (double) 587.33;
  final double NOTE_C5sharp = (double) 554.365;
  final double NOTE_C5 = (double) 523.251;
  final double NOTE_B4 = (double) 493.883;
  final double NOTE_A4sharp = (double) 466.164;
  final double NOTE_A4 = (double) 440;
  final double NOTE_G4sharp = (double) 415.305;
  final double NOTE_GS4 = (double) 415.305;
  final double NOTE_G4 = (double) 391.995;
  final double NOTE_F4sharp = (double) 369.994;
  final double NOTE_F4 = (double) 349.228;
  final double NOTE_E4 = (double) 329.628;
  final double NOTE_D4sharp = (double) 311.127;
  final double NOTE_DS4 = (double) 311.127;
  final double NOTE_D4 = (double) 293.665;
  final double NOTE_C4sharp = (double) 277.183;
  final double NOTE_C4 = (double) 261.626;

  // Fur Elise: https://en.wikipedia.org/wiki/F%C3%BCr_Elise
  double notes[] = { NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_C4,
      NOTE_E4, NOTE_A4, NOTE_B4, NOTE_E4, NOTE_GS4, NOTE_B4, NOTE_C5, NOTE_E4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5,
      NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_E4, NOTE_C5, NOTE_B4,
      NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_G4, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_F4, NOTE_E5, NOTE_D5,
      NOTE_C5, NOTE_E4, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_E4, NOTE_E5, NOTE_E4, NOTE_E5, NOTE_E4, NOTE_E5, NOTE_E4,
      NOTE_E5, NOTE_DS4, NOTE_E5, NOTE_D4, NOTE_E5, NOTE_DS4, NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_C4,
      NOTE_E4, NOTE_A4, NOTE_B4, NOTE_E4, NOTE_GS4, NOTE_B4, NOTE_C5, NOTE_E4, NOTE_E5, NOTE_DS5, NOTE_E5, NOTE_DS5,
      NOTE_E5, NOTE_B4, NOTE_D5, NOTE_C5, NOTE_A4, NOTE_C4, NOTE_E4, NOTE_A4, NOTE_B4, NOTE_E4, NOTE_C5, NOTE_B4,
      NOTE_A4, NOTE_OFF, };

  int durationsMs[] = { 111, 111, 111, 111, 111, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111,
      111, 111, 111, 111, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111,
      333, 111, 111, 111, 333, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111,
      111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111, 333, 111, 111, 111, 111, 111, 111, 111, 111, 111, 333, 111,
      111, 111, 333, 125, 125, 125, 999, 111, };

  double GetMusicFrequency(int dt_ms) {
    if (_timeoutMs >= 0) {
      /* still waiting for note duration */
      _timeoutMs -= dt_ms;
    } else if (_noteIter >= notes.length) {
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
}
