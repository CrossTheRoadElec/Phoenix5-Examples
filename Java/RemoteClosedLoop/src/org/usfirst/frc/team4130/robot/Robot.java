/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4130.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;


public class Robot extends IterativeRobot {

	/**
	 * Enum types for the various combinations of control modes and features.
	 * This can be replaced with function pointer, class interfaces, etc.
	 * However, this is meant to be a simple example using minimal language features.
	 */
	enum exampleType{
		kone_Axis_PercentOutput(1){
			@Override
			public exampleType previous(){
				return this;
			};
		},
		ktwo_Axis_PercentOutput(2),
		kone_Axis_Position(3),
		kone_Axis_Position_WithCustomFeedFwd(4),
		ktwo_Axis_Position(5),
		kone_Axis_Velocity(6),
		kone_Axis_Velocity_WithCustomFeedFwd(7),
		ktwo_Axis_Velocity(8),
		kone_Axis_MotionMagic(9),
		kone_Axis_MotionMagic_WithCustomFeedFwd(10),
		ktwo_Axis_MotionMagic(11),
		kone_Axis_MotionProfile(12),
		kone_Axis_MotionProfile_WithCustomFeedFwd(13),
		ktwo_Axis_MotionProfile(14),
		ktwo_Axis_MotionProfile_WithCustomFeedFwd(15),

		kTotalCount(16){
			@Override
			public exampleType next(){
				return this;
			};
		};
		
		public int value;
		exampleType(int value)
		{
			this.value = value;
		}
		
		public exampleType next(){
			return values()[ordinal() + 1];
		}
		public exampleType previous(){
			return values()[ordinal() - 1];
		}
	}
	
	
	exampleType _selectedExample = exampleType.kone_Axis_PercentOutput;
	exampleType _appliedExample = exampleType.kone_Axis_PercentOutput;
	
	enum ButtonEvent {
		ButtonOff, ButtonOffToOn, ButtonOn, ButtonOnToOff;
	}
	
	/* hardware objects */
	TalonSRX _talonLeft = new TalonSRX(6);
	TalonSRX _talonRght = new TalonSRX(2);
	TalonSRX _talonPigeon = new TalonSRX(5);
	PigeonIMU _imu = new PigeonIMU(_talonPigeon);
	Joystick _joy = new Joystick(0);
	
	/** Motion profile example manager*/
	MotionProfileExample  _motProfExample = new MotionProfileExample(_talonRght);

	/* a couple latched values to detect on-press events for buttons and POV */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	int _pov = 0;
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];
	
	/* depending on selected example, we must latch the targets. This is necessary
	 * so that the user is free to move the target stick and then lock in the target
	 * on button press. */
	double _target0 = 0;
	double _target1 = 0;
	
	/**
	 * This is different than robotInit - this is a method
	 * that we can call during disabledInit/teleopInit.
	 */
	void initRobot(){
		_talonRght.set(ControlMode.PercentOutput, 0);

		//------------ talons -----------------//
		_talonLeft.setInverted(false);
		_talonLeft.setSensorPhase(true);
		_talonRght.setInverted(true);
		_talonRght.setSensorPhase(true);

		//------------ setup filters -----------------//
		/* other side is quad */
		_talonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
												Constants.PID_PRIMARY,
												Constants.kTimeoutMs);

		/* Remote 0 will be the other side's Talon */
		_talonRght.configRemoteFeedbackFilter(	_talonLeft.getDeviceID(),
												RemoteSensorSource.TalonSRX_SelectedSensor,
												Constants.REMOTE_0,
												Constants.kTimeoutMs);
		/* Remote 1 will be a pigeon */
		_talonRght.configRemoteFeedbackFilter(	_talonPigeon.getDeviceID(),
												RemoteSensorSource.GadgeteerPigeon_Yaw,
												Constants.REMOTE_1,
												Constants.kTimeoutMs);
		/* setup sum and difference signals */
		_talonRght.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);
		_talonRght.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);
		_talonRght.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);
		_talonRght.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);
		/* select sum for distance(0), different for turn(1) */
		_talonRght.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);
		
		if (Constants.kHeadingSensorChoice == 0) {

			_talonRght.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference,
													Constants.PID_TURN,
													Constants.kTimeoutMs);

			/* do not scale down the primary sensor (distance) */
			_talonRght.configSelectedFeedbackCoefficient(1, Constants.PID_PRIMARY, Constants.kTimeoutMs);

			/* scale empirically measured units to 3600units, this gives us
			 * - 0.1 deg resolution
			 * - scales to human-readable units
			 * - keeps target away from overflow (12bit)
			 *
			 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
			 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
			 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
			 *  ... so at 3600 units per 360', that ensures 0.1 deg precision in firmware closed-loop
			 *  and motion profile trajectory points can range +-2 rotations.
			 */
			_talonRght.configSelectedFeedbackCoefficient(Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation, Constants.PID_TURN, Constants.kTimeoutMs);
		} else {

			/* do not scale down the primary sensor (distance).  If selected sensor is going to be a sensorSum
			 * user can pass 0.5 to produce an average. */
			_talonRght.configSelectedFeedbackCoefficient(1.0, Constants.PID_PRIMARY, Constants.kTimeoutMs);

			_talonRght.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1,
													Constants.PID_TURN,
													Constants.kTimeoutMs);

			_talonRght.configSelectedFeedbackCoefficient(Constants.kTurnTravelUnitsPerRotation / Constants.kPigeonUnitsPerRotation, Constants.PID_TURN, Constants.kTimeoutMs);
		}
		
		//------------ telemetry-----------------//
				_talonRght.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
				_talonRght.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
				_talonRght.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
				_talonRght.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.kTimeoutMs);
				/* speed up the left since we are polling it's sensor */
				_talonLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

				_talonLeft.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
				_talonRght.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

				_talonRght.configMotionAcceleration(1000, Constants.kTimeoutMs);
				_talonRght.configMotionCruiseVelocity(1000, Constants.kTimeoutMs);

				/* max out the peak output (for all modes).  However you can
				 * limit the output of a given PID object with configClosedLoopPeakOutput().
				 */
				_talonLeft.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
				_talonLeft.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
				_talonRght.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
				_talonRght.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

				/* distance servo */
				_talonRght.config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
				_talonRght.config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
				_talonRght.config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
				_talonRght.config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
				_talonRght.config_IntegralZone(Constants.kSlot_Distanc, (int)Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
				_talonRght.configClosedLoopPeakOutput(	Constants.kSlot_Distanc,
														Constants.kGains_Distanc.kPeakOutput,
														Constants.kTimeoutMs);

				/* turn servo */
				_talonRght.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
				_talonRght.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
				_talonRght.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
				_talonRght.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
				_talonRght.config_IntegralZone(Constants.kSlot_Turning, (int)Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
				_talonRght.configClosedLoopPeakOutput(	Constants.kSlot_Turning,
														Constants.kGains_Turning.kPeakOutput,
														Constants.kTimeoutMs);

				/* magic servo */
				_talonRght.config_kP(Constants.kSlot_MotProf, Constants.kGains_MotProf.kP, Constants.kTimeoutMs);
				_talonRght.config_kI(Constants.kSlot_MotProf, Constants.kGains_MotProf.kI, Constants.kTimeoutMs);
				_talonRght.config_kD(Constants.kSlot_MotProf, Constants.kGains_MotProf.kD, Constants.kTimeoutMs);
				_talonRght.config_kF(Constants.kSlot_MotProf, Constants.kGains_MotProf.kF, Constants.kTimeoutMs);
				_talonRght.config_IntegralZone(Constants.kSlot_MotProf, (int)Constants.kGains_MotProf.kIzone, Constants.kTimeoutMs);
				_talonRght.configClosedLoopPeakOutput(	Constants.kSlot_MotProf,
														Constants.kGains_MotProf.kPeakOutput,
														Constants.kTimeoutMs);

				/* velocity servo */
				_talonRght.config_kP(Constants.kSlot_Velocit, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
				_talonRght.config_kI(Constants.kSlot_Velocit, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
				_talonRght.config_kD(Constants.kSlot_Velocit, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
				_talonRght.config_kF(Constants.kSlot_Velocit, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
				_talonRght.config_IntegralZone(Constants.kSlot_Velocit, (int)Constants.kGains_Velocit.kIzone, Constants.kTimeoutMs);
				_talonRght.configClosedLoopPeakOutput(	Constants.kSlot_Velocit,
														Constants.kGains_Velocit.kPeakOutput,
														Constants.kTimeoutMs);

				_talonLeft.setNeutralMode(NeutralMode.Brake);
				_talonRght.setNeutralMode(NeutralMode.Brake);
				
				/* 1ms per loop.  PID loop can be slowed down if need be.
				 * For example,
				 * - if sensor updates are too slow
				 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
				 * - sensor movement is very slow causing the derivative error to be near zero.
				 */
				int closedLoopTimeMs = 1;
				_talonRght.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, Constants.PID_PRIMARY, Constants.kTimeoutMs);
				_talonRght.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, Constants.PID_TURN, Constants.kTimeoutMs);

				/**
				 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
				 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
				 */
				_talonRght.configAuxPIDPolarity(false, Constants.kTimeoutMs);
				
				zeroSensors();
	}
	
	void zeroSensors() {
		_talonLeft.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_talonRght.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_imu.setYaw(0, Constants.kTimeoutMs);
		_imu.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("        [Sensors] All sensors are zeroed.\n");
	}
	
	void neutralMotors(String reason) {
		_talonLeft.neutralOutput();
		_talonRght.neutralOutput();

		/* if caller is reporting WHY motors are being neutralized, report it */
		if (reason != null && reason.length() > 0) {
			System.out.print("  Motors are neutral, ");
			System.out.println(reason);
		}
	}
	
	void commonLoop() {
		/* temps for the first calls or button events */
		boolean bFirstCall = false;
		ButtonEvent bExecuteAction = ButtonEvent.ButtonOff;

		/* grab the joystick inputs */
		double joyFwd = -1 * _joy.getY(); /* positive stick => forward */
		double joyTurn = +1 * _joy.getTwist(); /* positive stick => right */
		double joyX = +1 * _joy.getX();
		int pov = _joy.getPOV();
		getButtons(btns);

		/* dead-band the sticks */
		joyFwd = Deadband(joyFwd);
		joyTurn = Deadband(joyTurn);

		/* look for new POV (top-hat or D-pad.)
		 * https://en.wikipedia.org/wiki/D-pad */
		if (_pov != pov) {
			_pov = pov;
			switch (_pov) {
				case 270: /* 270 deg, D-PAD L */
					if (_selectedExample.value > 0) {
						/* move up to the next selected example type */
						_selectedExample = _selectedExample.previous();
					}
					break;
				case 90: /* 90 deg, D-PAD R */
					if ((_selectedExample.value + 1) < exampleType.kTotalCount.value) {
						/* move up to the next selected example type */
						_selectedExample = _selectedExample.next();
					}
					break;

				case 0: /* 0 deg, D-PAD Up */
				{ /* enable neutral-if-remote-LOSS-OF-SIGNAL */
					double value = 0;
					_talonRght.configSetParameter(	ParamEnum.eRemoteSensorClosedLoopDisableNeutralOnLOS,
													value,
													0x00,
													0x00,
													Constants.kTimeoutMs);
					System.out.println("Enable neutral if remote sensor is unplugged (Loss of Signal) ");
				} break;

				case 180: /* 180 deg, D-PAD Down */
				{ /* disable neutral-if-remote-LOSS-OF-SIGNAL */
					double value = 1;
					_talonRght.configSetParameter(	ParamEnum.eRemoteSensorClosedLoopDisableNeutralOnLOS,
													value,
													0x00,
													0x00,
													Constants.kTimeoutMs);
					System.out.println("Disabled neutral if remote sensor is unplugged (Loss of Signal) ");
				} break;
			}
		}

		/* look for new button presses */
		if (btns[6] && !_btns[6]) { /* if button6 is just pressed */
			/* take note that user pressed the action button */
			bExecuteAction = ButtonEvent.ButtonOffToOn;
		} else if (!btns[6] && _btns[6]) {
			bExecuteAction = ButtonEvent.ButtonOnToOff;
		} else if (btns[6]) {
			bExecuteAction = ButtonEvent.ButtonOn;
		} else {
			bExecuteAction = ButtonEvent.ButtonOff;
		}
		/* button 1*/
		if (btns[1] && !_btns[1]) {
			/* if button1 is just pressed */
			zeroSensors();
		}
		CopyButtons(_btns, btns);

		/* user is holding shoulder button, execute selected example */
		if (_btns[5]) {
			/* button 5 is held down, jump to simple drive */
			if (_appliedExample != exampleType.kone_Axis_PercentOutput) {
				/* take note that this is the first loop of the new selection */
				bFirstCall = true;
				_appliedExample = exampleType.kone_Axis_PercentOutput;
			}
		} else if (_appliedExample != _selectedExample) {
			/* take note that this is the first loop of the new selection */
			bFirstCall = true;
			_appliedExample = _selectedExample;
		}

		switch (_appliedExample) {
			case kone_Axis_PercentOutput:
				one_Axis_PercentOutput(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case ktwo_Axis_PercentOutput:
				two_Axis_PercentOutput(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kone_Axis_Position:
				one_Axis_Position(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kone_Axis_Position_WithCustomFeedFwd:
				one_Axis_Position_WithCustomFeedFwd(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case ktwo_Axis_Position:
				two_Axis_Position(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kone_Axis_Velocity:
				one_Axis_Velocity(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kone_Axis_Velocity_WithCustomFeedFwd:
				one_Axis_Velocity_WithCustomFeedFwd(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case ktwo_Axis_Velocity:
				two_Axis_Velocity(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kone_Axis_MotionMagic:
				one_Axis_MotionMagic(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kone_Axis_MotionMagic_WithCustomFeedFwd:
				one_Axis_MotionMagic_WithCustomFeedFwd(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case ktwo_Axis_MotionMagic:
				two_Axis_MotionMagic(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kone_Axis_MotionProfile:
				one_Axis_MotionProfile(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kone_Axis_MotionProfile_WithCustomFeedFwd:
				one_Axis_MotionProfile_WithCustomFeedFwd(bFirstCall, bExecuteAction, joyFwd, joyTurn, joyX);
				break;
			case ktwo_Axis_MotionProfile:
				two_Axis_MotionProfile(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case ktwo_Axis_MotionProfile_WithCustomFeedFwd:
				two_Axis_MotionProfile_WithCustomFeedFwd(bFirstCall, bExecuteAction, joyFwd, joyTurn, joyX);
				break;
			default:
				neutralMotors("Invalid example selected\n");
				break;
		}
	}
	
	//------------------------- Loops -------------------------------//
		public void disabledInit() {
			/* initialize hardware so that sensor phases on set before Teleop.
			 * This makes self-test more useful. */
			initRobot();
		}
		public void disabledPeriodic() {
			commonLoop();
		}
		public void teleopInit() {
			/* initialize hardware at start of teleop, just in case Talon was replaced / field-upgraded during disable.
			 * All params are persistent except for status frame periods. */
			initRobot();
		}
		public void teleopPeriodic() {
			commonLoop();
		}
		
		
		//------------------------- Examples -------------------------------//
		void one_Axis_PercentOutput(boolean bFirstCall, ButtonEvent bExecuteAction, double joyY, double joyTurn) {

			/* calculate targets from gamepad inputs */
			double left = joyY + joyTurn;
			double rght = joyY - joyTurn;

			if (bFirstCall) {
				System.out.print("[00]one_Axis_PercentOutput selected, ");
				System.out.println("This is a basic arcade drove. ");
			}

			_talonLeft.set(ControlMode.PercentOutput, left);
			_talonRght.set(ControlMode.PercentOutput, rght);
		}
		void two_Axis_PercentOutput(boolean bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

			/* calculate targets from gamepad inputs */

			if (bFirstCall) {
				System.out.print("[01]two_Axis_PercentOutput selected, ");
				System.out.println("This is a basic arcade drove. ");
			}

			_talonLeft.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, +joyTurn);
			_talonRght.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, -joyTurn);
		}
		void one_Axis_Position(boolean bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

			/* calculate targets from gamepad inputs */
			double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;

			if (bFirstCall) {
				System.out.print("[02]one_Axis_Position selected, ");
				System.out.println("Press Button 6 to set target. ");
				neutralMotors("Target not set yet.\n");
				zeroSensors();

				_talonRght.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
			}

			if (bExecuteAction == ButtonEvent.ButtonOffToOn) {

			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				_target0 = target_sensorUnits;
				_talonRght.set(ControlMode.Position, _target0);
				_talonLeft.follow(_talonRght);
			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				//neutralMotors("Button let go\n");
			}
		}
		void one_Axis_Position_WithCustomFeedFwd(	boolean bFirstCall,
													ButtonEvent bExecuteAction,
													double joyForward,
													double joyTurn) {

			/* calculate targets from gamepad inputs */
			double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
			double feedFwdTerm = joyTurn * 0.25; /* how much motor output to add to the close loop output */

			if (bFirstCall) {
				System.out.print("[03]one_Axis_Position_WithCustomFeedFwd selected, ");
				System.out.println("Press Button 6 to set target. ");
				neutralMotors("Target not set yet.\n");
				zeroSensors();

				_talonRght.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
			}

			if (bExecuteAction == ButtonEvent.ButtonOffToOn) {

			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				_target0 = target_sensorUnits;
				_target1 = feedFwdTerm;
				_talonRght.set(ControlMode.Position, _target0, DemandType.ArbitraryFeedForward, _target1);
				_talonLeft.follow(_talonRght);
			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				//neutralMotors("Button let go\n");
			}

		}

		void two_Axis_Position(boolean bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

			/* calculate targets from gamepad inputs */
			double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
			double target_turn = joyTurn * Constants.kTurnTravelUnitsPerRotation * -1.0; /* positive right stick => negative heading target (turn to right) */

			if (bFirstCall) {
				System.out.print("[04]two_Axis_Position selected, ");
				System.out.println("Press Button 6 to set target. ");
				neutralMotors("Target not set yet.\n");
				zeroSensors();

				_talonRght.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
				_talonRght.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			}

			if (bExecuteAction == ButtonEvent.ButtonOffToOn) {

			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				_target0 = target_sensorUnits;
				_target1 = target_turn;
				_talonRght.set(ControlMode.Position, _target0, DemandType.AuxPID, _target1);
				_talonLeft.follow(_talonRght, FollowerType.AuxOutput1);
			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				//neutralMotors("Button let go\n");
			}

		}

		void one_Axis_Velocity(boolean bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

			/* calculate targets from gamepad inputs */
			double target_RPM = joyForward * 500; /* +- 500 RPM */
			double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;

			if (bFirstCall) {
				System.out.print("[05]one_Axis_Velocity selected, ");
				System.out.println("Press Button 6 to set target. ");
				neutralMotors("Target not set yet.\n");

				_talonRght.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
			}

			if (bExecuteAction == ButtonEvent.ButtonOffToOn) {

			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				_target0 = target_unitsPer100ms;
				_talonRght.set(ControlMode.Velocity, _target0);
				_talonLeft.follow(_talonRght);
			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				//neutralMotors("Button let go\n");
			}

		}
		void one_Axis_Velocity_WithCustomFeedFwd(	boolean bFirstCall,
													ButtonEvent bExecuteAction,
													double joyForward,
													double joyTurn) {

			/* calculate targets from gamepad inputs */
			double target_RPM = joyForward * 500; /* +- 500 RPM */
			double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;
			double feedFwdTerm = joyTurn * 0.25; /* how much to add to the close loop output */

			if (bFirstCall) {
				System.out.print("[06]one_Axis_Velocity_WithCustomFeedFwd selected, ");
				System.out.println("Press Button 6 to set target. ");
				neutralMotors("Target not set yet.\n");

				_talonRght.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
			}

			if (bExecuteAction == ButtonEvent.ButtonOffToOn) {

			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				_target0 = target_unitsPer100ms;
				_target1 = feedFwdTerm;
				_talonRght.set(ControlMode.Velocity, _target0, DemandType.ArbitraryFeedForward, _target1);
				_talonLeft.follow(_talonRght);
			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				//neutralMotors("Button let go\n");
			}

		}

		void two_Axis_Velocity(boolean bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

			/* calculate targets from gamepad inputs */
			double target_RPM = joyForward * 500; /* +- 500 RPM */
			double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;

			double heading_units = Constants.kTurnTravelUnitsPerRotation * joyTurn * -1.0; /* positive right stick => negative heading target (turn to right) */

			if (bFirstCall) {
				System.out.print("[07]two_Axis_Velocity selected, this can be used to approach a heading while mainting velocity. ");
				System.out.println("Press Button 6 to set target. ");
				neutralMotors("Target not set yet.\n");

				_talonRght.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
				_talonRght.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			}

			if (bExecuteAction == ButtonEvent.ButtonOffToOn) {

			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				_target0 = target_unitsPer100ms;
				_target1 = heading_units;
				_talonRght.set(ControlMode.Velocity, _target0, DemandType.AuxPID, _target1);
				_talonLeft.follow(_talonRght, FollowerType.AuxOutput1);
			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				//neutralMotors("Button let go\n");
			}

		}
		void one_Axis_MotionMagic(boolean bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

			/* calculate targets from gamepad inputs */
			double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;

			if (bFirstCall) {
				System.out.print("[08]one_Axis_MotionMagic selected, ");
				System.out.println("Press Button 6 to set target. ");
				neutralMotors("Target not set yet.\n");
				zeroSensors();

				_talonRght.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
			}

			if (bExecuteAction == ButtonEvent.ButtonOffToOn) {

			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				_target0 = target_sensorUnits;
				_talonRght.set(ControlMode.MotionMagic, _target0);
				_talonLeft.follow(_talonRght);
			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				//neutralMotors("Button let go\n");
			}

		}
		void one_Axis_MotionMagic_WithCustomFeedFwd(boolean bFirstCall,
													ButtonEvent bExecuteAction,
													double joyForward,
													double joyTurn) {

			/* calculate targets from gamepad inputs */
			double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
			double feedFwdTerm = joyTurn * 0.25;

			if (bFirstCall) {
				System.out.print("[09]one_Axis_MotionMagic_WithCustomFeedFwd selected, ");
				System.out.println("Press Button 6 to set target. ");
				neutralMotors("Target not set yet.\n");
				zeroSensors();

				_talonRght.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
			}

			if (bExecuteAction == ButtonEvent.ButtonOffToOn) {

			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				_target0 = target_sensorUnits;
				_target1 = feedFwdTerm;
				_talonRght.set(ControlMode.MotionMagic, _target0, DemandType.ArbitraryFeedForward, _target1);
				_talonLeft.follow(_talonRght);
			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				//neutralMotors("Button let go\n");
			}

		}
		void two_Axis_MotionMagic(boolean bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

			/* calculate targets from gamepad inputs */
			double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
			double heading_units = joyTurn * Constants.kTurnTravelUnitsPerRotation * -1.0; /* positive right stick => negative heading target (turn to right) */

			if (bFirstCall) {
				System.out.print("[10]two_Axis_MotionMagic selected, ");
				System.out.println("Press Button 6 to set target. ");
				neutralMotors("Target not set yet.\n");
				zeroSensors();
				_talonRght.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
				_talonRght.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			}

			if (bExecuteAction == ButtonEvent.ButtonOffToOn) {

			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				_target0 = target_sensorUnits;
				_target1 = heading_units;

				_talonRght.set(ControlMode.MotionMagic, _target0, DemandType.AuxPID, _target1);
				_talonLeft.follow(_talonRght, FollowerType.AuxOutput1);
			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {
				//neutralMotors("Button let go\n");
			}
		}
		void one_Axis_MotionProfile(boolean bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

			/* calculate targets from gamepad inputs */
			boolean bMoveForward = (joyForward >= 0) ? true : false;

			if (bFirstCall) {
				System.out.print("[11]one_Axis_MotionProfile selected, ");
				System.out.println("Press Button 6 to fire the profile. ");
				neutralMotors("Target not set yet.\n");

				/* slots are selected in the profile, not via selectProfileSlot() */

			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {


			} else if (bExecuteAction == ButtonEvent.ButtonOffToOn) {

				neutralMotors("Button let go\n");
				zeroSensors();
				_motProfExample.reset();
				_motProfExample.start(0, bMoveForward); /*final target heading is ignored, doesn't matter */


			} else if (bExecuteAction == ButtonEvent.ButtonOn) {

				_talonRght.set(ControlMode.MotionProfile, _motProfExample.getSetValue().value);
				_talonLeft.follow(_talonRght);
			}

			/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
			_motProfExample.control();
		}
		void one_Axis_MotionProfile_WithCustomFeedFwd(	boolean bFirstCall,
														ButtonEvent bExecuteAction,
														double joyForward,
														double joyTurn,
														double joyX) {
			/* calculate targets from gamepad inputs */
			double feedFwdTerm = joyX * 0.25;
			boolean bMoveForward = (joyForward >= 0) ? true : false;

			if (bFirstCall) {
				System.out.print("[12]one_Axis_MotionProfile_WithCustomFeedFwd selected, ");
				System.out.println("Press Button 6 to fire the profile. ");
				neutralMotors("Target not set yet.\n");

				/* slots are selected in the profile, not via selectProfileSlot() */

			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {

			} else if (bExecuteAction == ButtonEvent.ButtonOffToOn) {
				neutralMotors("Button let go\n");
				zeroSensors();
				_motProfExample.reset();
				_motProfExample.start(0, bMoveForward); /*final target heading is ignored, doesn't matter */
			} else if (bExecuteAction == ButtonEvent.ButtonOn) {

				_talonRght.set(	ControlMode.MotionProfile,
								_motProfExample.getSetValue().value,
								DemandType.ArbitraryFeedForward,
								feedFwdTerm);
				_talonLeft.follow(_talonRght);
			}
			/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
			_motProfExample.control();

		}
		void two_Axis_MotionProfile(boolean bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

			/* calculate targets from gamepad inputs */
			boolean bMoveForward = (joyForward >= 0) ? true : false;
			double finalHeading_units = Constants.kTurnTravelUnitsPerRotation * joyTurn * -1.0; /* positive right stick => negative heading target (turn to right) */

			if (bFirstCall) {
				System.out.print("[13]two_Axis_MotionProfile selected, ");
				System.out.println("Press Button 6 to fire the profile. ");
				neutralMotors("Target not set yet.\n");

				/* slots are selected in the profile, not via selectProfileSlot() */

			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {

			} else if (bExecuteAction == ButtonEvent.ButtonOffToOn) {
				neutralMotors("Button let go\n");
				zeroSensors();
				_motProfExample.reset();
				_motProfExample.start(finalHeading_units, bMoveForward);
			} else if (bExecuteAction == ButtonEvent.ButtonOn) {

				_talonRght.set(ControlMode.MotionProfileArc, _motProfExample.getSetValue().value);
				_talonLeft.follow(_talonRght, FollowerType.AuxOutput1);
			}
			/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
			_motProfExample.control();

		}
		void two_Axis_MotionProfile_WithCustomFeedFwd(	boolean bFirstCall,
														ButtonEvent bExecuteAction,
														double joyForward,
														double joyTurn,
														double joyX) {
			/* calculate targets from gamepad inputs */
			double feedFwdTerm = joyX * 0.25;
			boolean bMoveForward = (joyForward >= 0) ? true : false;
			double finalHeading_units = Constants.kTurnTravelUnitsPerRotation * joyTurn * -1.0; /* positive right stick => negative heading target (turn to right) */

			if (bFirstCall) {
				System.out.print("[14]two_Axis_MotionProfile_WithCustomFeedFwd selected, ");
				System.out.println("Press Button 6 to fire the profile. ");
				neutralMotors("Target not set yet.\n");

				/* slots are selected in the profile, not via selectProfileSlot() */

			} else if (bExecuteAction == ButtonEvent.ButtonOnToOff) {

			} else if (bExecuteAction == ButtonEvent.ButtonOffToOn) {
				neutralMotors("Button let go\n");
				zeroSensors();
				_motProfExample.reset();
				_motProfExample.start(finalHeading_units, bMoveForward);
			} else if (bExecuteAction == ButtonEvent.ButtonOn) {
				_talonRght.set(	ControlMode.MotionProfileArc,
								_motProfExample.getSetValue().value,
								DemandType.ArbitraryFeedForward,
								feedFwdTerm);
				_talonLeft.follow(_talonRght, FollowerType.AuxOutput1);
			}

			/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
			_motProfExample.control();
		}
	
	
	//-------------- Some helpful routines ---------------//
		void getButtons(boolean[] btns) {
			for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
				btns[i] = _joy.getRawButton(i);
			}
		}
		void CopyButtons(boolean[] destination, boolean[] source) {
			for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
				destination[i] = source[i];
			}
		}
		double Deadband(double value) {
			if (value >= +0.05) {
				return value;
			}
			if (value <= -0.05) {
				return value;
			}
			return 0;
		}
}
