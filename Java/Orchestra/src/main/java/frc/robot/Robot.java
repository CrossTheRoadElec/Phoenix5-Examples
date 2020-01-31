package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Top level Robot class 
 */
public class Robot extends TimedRobot {

    /* The orchestra object that holds all the instruments */
    Orchestra _orchestra;

    /* Talon FXs to play music through.  
    More complex music MIDIs will contain several tracks, requiring multiple instruments. */
    TalonFX [] _fxes =  { new TalonFX(0), new TalonFX(1) };

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
        /* A list of TalonFX's that are to be used as instruments */
        ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();
      
        /* Initialize the TalonFX's to be used */
        for (int i = 0; i < _fxes.length; ++i) {
            _instruments.add(_fxes[i]);
        }
        /* Create the orchestra with the TalonFX instruments */
        _orchestra = new Orchestra(_instruments);
        _joy = new Joystick(0);
    }

    /** @return 0 if no button pressed, index of button otherwise. */
    int getButton() {
        for (int i = 1; i < 9; ++i) {
            if (_joy.getRawButton(i)) {
                return i;
            }
        }
        return 0;
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        /* poll gamepad */
        int btn = getButton();
        int currentPOV = _joy.getPOV();


        if (_lastButton != btn) {
            _lastButton = btn;

            switch (btn) {
            case 1:
                if (_orchestra.isPlaying())
                    _orchestra.pause();
                else
                    _orchestra.play();
                break;
            case 2:
                _orchestra.pause();
                System.out.println("Paused");
                break;
            case 3:
                _orchestra.stop();
                System.out.println("Stopped");
                break;
            }
        }

        if (_lastPOV != currentPOV) {
            _lastPOV = currentPOV;

            switch (currentPOV) {
            case 90:
                /* increment song selection */
                if (++_songSelection >= _songs.length) {
                    _songSelection = 0;
                }
                System.out.println("Song selected is: " + _songs[_songSelection]);
                _orchestra.loadMusic(_songs[_songSelection]);
                break;
            case 270:
                /* decrement song selection */
                if (--_songSelection < 0) {
                    _songSelection = _songs.length - 1;
                }
                System.out.println("Song selected is: " + _songs[_songSelection]);
                _orchestra.loadMusic(_songs[_songSelection]);
                break;
            }
        }
    }
}
