/**
 * Example demonstrating the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 * 
 * Be sure to select the correct feedback sensor using SetFeedbackDevice() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the reverseSensor() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target position.  
 *
 * Tweak the PID gains accordingly.
 */
package org.usfirst.frc.team217.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;

public class Robot extends IterativeRobot {
  
	TalonSRX _talon = new TalonSRX(3);	
	Joystick _joy = new Joystick(0);	
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	boolean _lastButton1 = false;
	/** save the target position to servo to */
	double targetPositionRotations;
	
	public void robotInit() {
		/* lets grab the 360 degree position of the MagEncoder's absolute position */
		int absolutePosition = _talon.getSelectedSensorPosition(Constants.kTimeoutMs) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
        /* use the low level API to set the quad encoder signal */
        _talon.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        
        /* choose the sensor and sensor direction */
        _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        _talon.setSensorPhase(true);
        //_talon.configEncoderCodesPerRev(XXX), // if using FeedbackDevice.QuadEncoder
        //_talon.configPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

        /* set the peak and nominal outputs, 12V means full */
        _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
        _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
        _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
        /* set the allowable closed-loop error,
         * Closed-Loop output will be neutral within this range.
         * See Table in Section 17.2.1 for native units per rotation. 
         */
        _talon.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); /* always servo */
        /* set closed loop gains in slot0 */
        _talon.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
        _talon.config_kP(Constants.kPIDLoopIdx, 0.1, Constants.kTimeoutMs);
        _talon.config_kI(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
        _talon.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);

	}
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	/* get gamepad axis */
    	double leftYstick = _joy.getY();
    	double motorOutput = _talon.getMotorOutputPercent();
    	boolean button1 = _joy.getRawButton(1);
    	boolean button2 = _joy.getRawButton(2);
    	/* prepare line to print */
		_sb.append("\tout:");
		_sb.append(motorOutput);
        _sb.append("\tpos:");
        _sb.append(_talon.getSelectedSensorPosition(0) );
        /* on button1 press enter closed-loop mode on target position */
        if(!_lastButton1 && button1) {
        	/* Position mode - button just pressed */
        	targetPositionRotations = leftYstick * 50.0 * 4096; /* 50 Rotations * 4096 u/rev in either direction */
        	_talon.set(ControlMode.Position, targetPositionRotations); /* 50 rotations in either direction */

        }
        /* on button2 just straight drive */
        if(button2) {
        	/* Percent voltage mode */
        	_talon.set(ControlMode.PercentOutput, leftYstick);
        }
        /* if Talon is in position closed-loop, print some more info */
        if( _talon.getControlMode() == ControlMode.Position) {
        	/* append more signals to print when in speed mode. */
        	_sb.append("\terrNative:");
        	_sb.append(_talon.getClosedLoopError(0));
        	_sb.append("\ttrg:");
        	_sb.append(targetPositionRotations);
        }
        /* print every ten loops, printing too much too fast is generally bad for performance */ 
        if(++_loops >= 10) {
        	_loops = 0;
        	System.out.println(_sb.toString());
        }
        _sb.setLength(0);
        /* save button state for on press detect */
        _lastButton1 = button1;
    }
}
