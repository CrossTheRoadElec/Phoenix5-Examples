/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /* Total number of instruments in the orchestra */
  final int INSTRUMENT_COUNT = 5;

  /* The orchestra object that holds all the instruments */
  Orchestra _orchestra;

  /* A list of TalonFX's that are to be used as instruments */
  ArrayList<TalonFX> _instruments;

  /* An array of songs that are available to be played */
  String[] _songs = new String[] {
    "song1.chrp",
    "song2.chrp",
    "song3.chrp",
    "song4.chrp",
    "song5.chrp",
    "song6.chrp",
    "song7.chrp",
    "song8.chrp",
    "song9.chrp",
    "song10.chrp",
    "song11.chrp",
  };
  int _songSelection = 0;

  Joystick _joy;
  int _lastButton = 0;
  int _lastPOV = 0;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    /* Initialize the TalonFX's to be used */
    _instruments = new ArrayList<TalonFX>();
    for(int i = 0; i < INSTRUMENT_COUNT; ++i) {
      _instruments.add(new TalonFX(i + 1));
    }
    /* Create the orchestra with the TalonFX instruments */
    _orchestra = new Orchestra(_instruments);
    _joy = new Joystick(0);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    if(_joy.getRawButton(1) && _lastButton != 1) {
      /* Play the song */
      _orchestra.play();
      _lastButton = 1;

      System.out.println("Playing");
    }
    if(_joy.getRawButton(2) && _lastButton != 2) {
      /* Pause */
      _orchestra.pause();
      _lastButton = 2;

      System.out.println("Paused");
    }
    if(_joy.getRawButton(3) && _lastButton != 3) {
      /* Stop */
      _orchestra.stop();
      _lastButton = 3;

      System.out.println("Stopped");
    }
    if(_joy.getRawButton(4) && _lastButton != 4) {
      /* Load selected song */
      _orchestra.loadMusic(_songs[_songSelection]);
      _lastButton = 4;

      System.out.println("Loaded " + _songs[_songSelection]);
    }

    int currentPOV = _joy.getPOV();
    if(currentPOV == 90 && _lastPOV != 90) {
      _songSelection++;
      if(_songSelection >= _songs.length) _songSelection = 0;

      System.out.println("Song selected is: " + _songs[_songSelection]);
    }
    if(currentPOV == 270 && _lastPOV != 270) {
      _songSelection--;
      if(_songSelection < 0) _songSelection = _songs.length - 1;
      
      System.out.println("Song selected is: " + _songs[_songSelection]);
    }
    _lastPOV = currentPOV;
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
