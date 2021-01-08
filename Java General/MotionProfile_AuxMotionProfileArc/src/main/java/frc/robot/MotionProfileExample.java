/**
 * Example logic for firing and managing motion profiles.
 * This example sends MPs, waits for them to finish
 * Although this code uses a Talon SRX, nowhere Set() Called. 
 * This is done in Robot.java to demonstrate how to change control modes on the fly.
 * 
 * The only routines we call on Talon are....
 * changeMotionControlFramePeriod
 * 
 * getMotionProfileStatus()			// Get MP Status from MC
 * clearMotionProfileHasUnderrun()	// Clear MP 
 * pushMotionProfileTrajectory()
 * clearMotionProfileTrajectories()
 * processMotionProfileBuffer()  	// To push/clear, and process the trajectory points.
 * getControlMode,					// To check if we are in Motion Profile Control mode.
 * 
 * Example of advanced features not demonstrated here...
 * [1] Calling pushMotionProfileTrajectory() continuously while the Talon executes the motion profile, thereby keeping it going indefinitely.
 * [2] Instead of setting the sensor position to zero at the start of each MP, the program could offset the MP's position based on current position. 
 */
package frc.robot;

import edu.wpi.first.wpilibj.Notifier;

import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;

public class MotionProfileExample {
	/**
	 * The status of the motion profile executer and buffer inside the Talon.
	 * Instead of creating a new one every time we call getMotionProfileStatus,
	 * keep one copy.
	 */
	private MotionProfileStatus _status = new MotionProfileStatus();

	/** additional cache for holding the active trajectory point */
	double _pos=0,_vel=0,_heading=0;

	double _endHeading = 0;
	
	/**
	 * reference to the talon we plan on manipulating. We will not changeMode()
	 * or call set(), just get motion profile status and make decisions based on
	 * motion profile.
	 */
	private IMotorController _motorController;
	/**
	 * State machine to make sure we let enough of the motion profile stream to
	 * talon before we fire it.
	 */
	private int _state = 0;
	/**
	 * Any time you have a state machine that waits for external events, its a
	 * good idea to add a timeout. Set to -1 to disable. Set to nonzero to count
	 * down to '0' which will print an error message. Counting loops is not a
	 * very accurate method of tracking timeout, but this is just conservative
	 * timeout. Getting time-stamps would certainly work too, this is just
	 * simple (no need to worry about timer overflows).
	 */
	private int _loopTimeout = -1;
	/**
	 * If start() gets called, this flag is set and in the control() we will
	 * service it.
	 */
	private boolean _bStart = false;
	
	private boolean _bForward = false;

	/**
	 * Since the CANTalon.set() routine is mode specific, deduce what we want
	 * the set value to be and let the calling module apply it whenever we
	 * decide to switch to MP mode.
	 */
	private SetValueMotionProfile _setValue = SetValueMotionProfile.Disable;
	/**
	 * How many trajectory points do we wait for before firing the motion
	 * profile.
	 */
	private static final int kMinPointsInTalon = 20;
	/**
	 * Just a state timeout to make sure we don't get stuck anywhere. Each loop
	 * is about 20ms.
	 */
	private static final int kNumLoopsTimeout = 10;
	
	/**
	 * Lets create a periodic task to funnel our trajectory points into our talon.
	 * It doesn't need to be very accurate, just needs to keep pace with the motion
	 * profiler executer.  Now if you're trajectory points are slow, there is no need
	 * to do this, just call _talon.processMotionProfileBuffer() in your teleop loop.
	 * Generally speaking you want to call it at least twice as fast as the duration
	 * of your trajectory points.  So if they are firing every 20ms, you should call 
	 * every 10ms.
	 */
	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {  _motorController.processMotionProfileBuffer();    }
	}
	Notifier _notifer = new Notifier(new PeriodicRunnable());
	

	/**
	 * C'tor
	 * 
	 * @param talon
	 *            reference to Talon object to fetch motion profile status from.
	 */
	public MotionProfileExample(IMotorController motorController) {
		_motorController = motorController;
		/*
		 * since our MP is 10ms per point, set the control frame rate and the
		 * notifer to half that
		 */
		_motorController.changeMotionControlFramePeriod(5);
		_notifer.startPeriodic(0.005);
	}

	/**
	 * Called to clear Motion profile buffer and reset state info during
	 * disabled and when Talon is not in MP control mode.
	 */
	public void reset() {
		/*
		 * Let's clear the buffer just in case user decided to disable in the
		 * middle of an MP, and now we have the second half of a profile just
		 * sitting in memory.
		 */
		_motorController.clearMotionProfileTrajectories();
		/* When we do re-enter motionProfile control mode, stay disabled. */
		_setValue = SetValueMotionProfile.Disable;
		/* When we do start running our state machine start at the beginning. */
		_state = 0;
		_loopTimeout = -1;
		/*
		 * If application wanted to start an MP before, ignore and wait for next
		 * button press
		 */
		_bStart = false;
	}
	
	boolean IsMotionProfile(ControlMode controlMode) {
		if (controlMode == ControlMode.MotionProfile)
			return true;
		if (controlMode == ControlMode.MotionProfileArc)
			return true;
		return false;
	}

	/**
	 * Called every loop.
	 */
	public void control() {
		/**
		 * Track time, this is rudimentary but that's okay, 
		 * we just want to make sure things never get stuck.
		 */
		if (_loopTimeout < 0) {
			/* Do nothing, timeout is disabled */
		} else {
			/* Timeout is nonzero */
			if (_loopTimeout == 0) {
				/**
				 * Something is wrong. 
				 * Talon is not present, unplugged, breaker tripped
				 */
				Instrumentation.OnNoProgress();
			} else {
				--_loopTimeout;
			}
		}

		/* first check if we are in MP mode */
		if (false == IsMotionProfile(_motorController.getControlMode())) {
			/*
			 * we are not in MP mode. We are probably driving the robot around
			 * using gamepads or some other mode.
			 */
			_state = 0;
			_loopTimeout = -1;
		} else {
			/*
			 * we are in MP control mode. That means: starting Mps, checking Mp
			 * progress, and possibly interrupting MPs if thats what you want to
			 * do.
			 */
			switch (_state) {
				case 0: /* wait for application to tell us to start an MP */
					if (_bStart) {
						_bStart = false;
	
						_setValue = SetValueMotionProfile.Disable;
						startFilling();
						/*
						 * MP is being sent to CAN bus, wait a small amount of time
						 */
						_state = 1;
						_loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 1: 
					_motorController.getMotionProfileStatus(_status);
					/*
					 * wait for MP to stream to Talon, really just the first few
					 * points
					 */
					/* do we have a minimum numberof points in Talon */
					if (_status.btmBufferCnt > kMinPointsInTalon) {
						/* start (once) the motion profile */
						_setValue = SetValueMotionProfile.Enable;
						/* MP will start once the control frame gets scheduled */
						_state = 2;
						_loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 2: /* check the status of the MP */

					/* Get the motion profile status every loop */
					_motorController.getMotionProfileStatus(_status);
					/*
					 * if talon is reporting things are good, keep adding to our
					 * timeout. Really this is so that you can unplug your talon in
					 * the middle of an MP and react to it.
					 */
					if (_status.isUnderrun == false) {
						_loopTimeout = kNumLoopsTimeout;
					}
					/*
					 * If we are executing an MP and the MP finished, start loading
					 * another. We will go into hold state so robot servo's
					 * position.
					 */
					if (_status.activePointValid && _status.isLast) {
						/*
						 * because we set the last point's isLast to true, we will
						 * get here when the MP is done
						 */
						_setValue = SetValueMotionProfile.Hold;
						_state = 0;
						_loopTimeout = -1;
					}
					break;
			}

			/* Get the motion profile status every loop */
			_heading = _motorController.getActiveTrajectoryPosition(1);
			_pos = _motorController.getActiveTrajectoryPosition();
			_vel = _motorController.getActiveTrajectoryVelocity();

			/* printfs and/or logging */
			Instrumentation.process(_status, _pos, _vel, _heading);
		}
	}

	/** Start filling the MPs to all of the involved Talons. */
	private void startFilling() {
		/* since this example only has one talon, just update that one */
		startFilling(MotionProfile.Points, MotionProfile.kNumPoints);
	}

	private void startFilling(double[][] profile, int totalCnt) {

		/* create an empty point */
		TrajectoryPoint point = new TrajectoryPoint();

		/* did we get an underrun condition since last time we checked ? */
		if (_status.hasUnderrun) {
			/* better log it so we know about it */
			Instrumentation.OnUnderrun();
			/*
			 * clear the error. This flag does not auto clear, this way 
			 * we never miss logging it.
			 */
			_motorController.clearMotionProfileHasUnderrun(Constants.kTimeoutMs);
		}
		/*
		 * just in case we are interrupting another MP and there is still buffer
		 * points in memory, clear it.
		 */
		_motorController.clearMotionProfileTrajectories();

		/* set the base trajectory period to zero, use the individual trajectory period below */
		_motorController.configMotionProfileTrajectoryPeriod(Constants.kBaseTrajPeriodMs, Constants.kTimeoutMs);
		
		/* squirell away the final target distance, we will use this for heading generation */
		double finalPositionRot = profile[totalCnt-1][0];
		
		/* This is fast since it's just into our TOP buffer */
		for (int i = 0; i < totalCnt; ++i) {
			double direction = _bForward ? +1 : -1;
			double positionRot = profile[i][0];
			double velocityRPM = profile[i][1];
			double heading = _endHeading * positionRot / finalPositionRot; /* scale heading progress to position progress */

			/* for each point, fill our structure and pass it to API */
			point.position = direction * positionRot * Constants.kSensorUnitsPerRotation * 2; 		//Convert Revolutions to Units
			point.velocity = direction * velocityRPM * Constants.kSensorUnitsPerRotation / 600.0; 	//Convert RPM to Units/100ms
			point.auxiliaryPos = heading; /* scaled such that 3600 => 360 deg */
			point.profileSlotSelect0 = Constants.kSlot_MotProf; /* which set of gains would you like to use [0,3]? */
			point.profileSlotSelect1 = Constants.kSlot_Turning; /* auxiliary PID [0,1], leave zero */
			point.timeDur = (int)profile[i][2];		// Trajectory Duration is no longer a thing...
			point.zeroPos = false;
			if (i == 0)
				point.zeroPos = true; /* set this to true on the first point */
			point.useAuxPID = true;

			point.isLastPoint = false;
			if ((i + 1) == totalCnt)
				point.isLastPoint = true; /* set this to true on the last point  */

			_motorController.pushMotionProfileTrajectory(point);
		}
	}

	/**
	 * Called by application to signal Talon to start the buffered MP (when it's
	 * able to).
	 */
	void start(double endHeading, boolean bForward) {
		_bStart = true;
		_bForward = bForward;
		_endHeading = endHeading;
	}

	/**
	 * @return the output value to pass to Talon's set() routine. 0 for disable
	 *         motion-profile output, 1 for enable motion-profile, 2 for hold
	 *         current motion profile trajectory point.
	 */
	SetValueMotionProfile getSetValue() {
		return _setValue;
	}
}
