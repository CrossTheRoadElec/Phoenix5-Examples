/**
 * Example demonstrating the remote sensor features of CTRE Products.
 * Use a Logitech gamepad in D-Input mode.
 * There are many example use-cases demonstrated here including...
 * - Using the SUM and DIFFERENCE Remote sensors features for robots that have a left and right encoder.
 * - Using the second auxiliary PID loop for maintaining robot heading (via Pigeon or right-minus-left encoder).
 * - Enhanced configuration of PID loops
 * - adding a constant feed forward to single-axis position/velocity/motion-magic.
 * - adding a constant feed forward to single and two-axis motion-profiles.
 *
 */
#include <iostream>
#include <string>
#include <unistd.h>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"

#include "Joystick.h"
#include "MotionProfileExample.h"

class Robot: public frc::IterativeRobot {
public:
	/**
	 * Enum types for the various combinations of control modes and features.
	 * This can be replaced with function pointer, class interfaces, etc.
	 * However, this is meant to be a simple example using minimal language features.
	 */
	enum ExampleType
		: uint32_t {
			kOne_Axis_PercentOutput,
		kTwo_Axis_PercentOutput,
		kOne_Axis_Position,
		kOne_Axis_Position_WithCustomFeedFwd,
		kTwo_Axis_Position,
		kOne_Axis_Velocity,
		kOne_Axis_Velocity_WithCustomFeedFwd,
		kTwo_Axis_Velocity,
		kOne_Axis_MotionMagic,
		kOne_Axis_MotionMagic_WithCustomFeedFwd,
		kTwo_Axis_MotionMagic,
		kOne_Axis_MotionProfile,
		kOne_Axis_MotionProfile_WithCustomFeedFwd,
		kTwo_Axis_MotionProfile,
		kTwo_Axis_MotionProfile_WithCustomFeedFwd,

		kTotalCount,
	};
	ExampleType _selectedExample = ExampleType::kOne_Axis_PercentOutput;
	ExampleType _appliedExample = ExampleType::kOne_Axis_PercentOutput;

	enum ButtonEvent {
		ButtonOff, ButtonOffToOn, ButtonOn, ButtonOnToOff,
	};

	/* hardware objects */
	TalonSRX & _talonLeft = *new TalonSRX(6);
	TalonSRX &_talonRght = *new TalonSRX(2);
	TalonSRX &_talonPigeon = *new TalonSRX(5);
	PigeonIMU & _imu = *new PigeonIMU(&_talonPigeon);
	Joystick &_joy = *new Joystick(0);

	/** Motion profile example manager*/
	MotionProfileExample * _motProfExample = new MotionProfileExample(_talonRght);

	/* a couple latched values to detect on-press events for buttons and POV */
	bool _btns[Constants.kNumButtonsPlusOne];
	int _pov = 0;

	/* depending on selected example, we must latch the targets. This is necessary
	 * so that the user is free to move the target stick and then lock in the target
	 * on button press. */
	double _target0 = 0;
	double _target1 = 0;

	void InitRobot() {

		_talonRght.Set(ControlMode::PercentOutput, 0);

		//------------ talons -----------------//
		_talonLeft.SetInverted(false);
		_talonLeft.SetSensorPhase(true);
		_talonRght.SetInverted(true);
		_talonRght.SetSensorPhase(true);

		//------------ setup filters -----------------//
		/* other side is quad */
		_talonLeft.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,
												Constants.PID_PRIMARY,
												Constants.kTimeoutMs);

		/* Remote 0 will be the other side's Talon */
		_talonRght.ConfigRemoteFeedbackFilter(	_talonLeft.GetDeviceID(),
												RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor,
												Constants.REMOTE_0,
												Constants.kTimeoutMs);
		/* Remote 1 will be a pigeon */
		_talonRght.ConfigRemoteFeedbackFilter(	_talonPigeon.GetDeviceID(),
												RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw,
												Constants.REMOTE_1,
												Constants.kTimeoutMs);
		/* setup sum and difference signals */
		_talonRght.ConfigSensorTerm(SensorTerm::SensorTerm_Sum0, FeedbackDevice::RemoteSensor0, Constants.kTimeoutMs);
		_talonRght.ConfigSensorTerm(SensorTerm::SensorTerm_Sum1, FeedbackDevice::QuadEncoder, Constants.kTimeoutMs);
		_talonRght.ConfigSensorTerm(SensorTerm::SensorTerm_Diff1, FeedbackDevice::RemoteSensor0, Constants.kTimeoutMs);
		_talonRght.ConfigSensorTerm(SensorTerm::SensorTerm_Diff0, FeedbackDevice::QuadEncoder, Constants.kTimeoutMs);
		/* select sum for distance(0), different for turn(1) */
		_talonRght.ConfigSelectedFeedbackSensor(FeedbackDevice::SensorSum, Constants.PID_PRIMARY, Constants.kTimeoutMs);
		if (Constants.kHeadingSensorChoice == 0) {

			_talonRght.ConfigSelectedFeedbackSensor(FeedbackDevice::SensorDifference,
													Constants.PID_TURN,
													Constants.kTimeoutMs);

			/* do not scale down the primary sensor (distance) */
			_talonRght.ConfigSelectedFeedbackCoefficient(1, Constants.PID_PRIMARY, Constants.kTimeoutMs);

			/* scale empirically measured units to 3600units, this gives us
			 * - 0.1 deg resolution
			 * - scales to human-readable units
			 * - keeps target away from overflow (12bit)
			 *
			 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
			 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
			 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
			 *  ... so at 3600 units per 360', that ensures 0.1 deg precision in firmware closed-loop
			 *  and motion profile trajectory points can range ±2 rotations.
			 */
			_talonRght.ConfigSelectedFeedbackCoefficient(Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation, Constants.PID_TURN, Constants.kTimeoutMs);
		} else {

			/* do not scale down the primary sensor (distance).  If selected sensor is going to be a sensorSum
			 * user can pass 0.5 to produce an average. */
			_talonRght.ConfigSelectedFeedbackCoefficient(1.0, Constants.PID_PRIMARY, Constants.kTimeoutMs);

			_talonRght.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor1,
													Constants.PID_TURN,
													Constants.kTimeoutMs);

			_talonRght.ConfigSelectedFeedbackCoefficient(Constants.kTurnTravelUnitsPerRotation / Constants.kPigeonUnitsPerRotation, Constants.PID_TURN, Constants.kTimeoutMs);
		}
		//------------ telemetry-----------------//
		_talonRght.SetStatusFramePeriod(StatusFrame::Status_12_Feedback1_, 20, Constants.kTimeoutMs);
		_talonRght.SetStatusFramePeriod(StatusFrame::Status_13_Base_PIDF0_, 20, Constants.kTimeoutMs);
		_talonRght.SetStatusFramePeriod(StatusFrame::Status_14_Turn_PIDF1_, 20, Constants.kTimeoutMs);
		_talonRght.SetStatusFramePeriod(StatusFrame::Status_10_Targets_, 20, Constants.kTimeoutMs);
		/* speed up the left since we are polling it's sensor */
		_talonLeft.SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 5, Constants.kTimeoutMs);

		_talonLeft.ConfigNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		_talonRght.ConfigNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

		_talonRght.ConfigMotionAcceleration(1000, Constants.kTimeoutMs);
		_talonRght.ConfigMotionCruiseVelocity(1000, Constants.kTimeoutMs);

		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with ConfigClosedLoopPeakOutput().
		 */
		_talonLeft.ConfigPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_talonLeft.ConfigPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		_talonRght.ConfigPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_talonRght.ConfigPeakOutputReverse(-1.0, Constants.kTimeoutMs);

		/* distance servo */
		_talonRght.Config_kP(Constants.kSlot_Distanc, Constants.kGains_Distanc.kP, Constants.kTimeoutMs);
		_talonRght.Config_kI(Constants.kSlot_Distanc, Constants.kGains_Distanc.kI, Constants.kTimeoutMs);
		_talonRght.Config_kD(Constants.kSlot_Distanc, Constants.kGains_Distanc.kD, Constants.kTimeoutMs);
		_talonRght.Config_kF(Constants.kSlot_Distanc, Constants.kGains_Distanc.kF, Constants.kTimeoutMs);
		_talonRght.Config_IntegralZone(Constants.kSlot_Distanc, Constants.kGains_Distanc.kIzone, Constants.kTimeoutMs);
		_talonRght.ConfigClosedLoopPeakOutput(	Constants.kSlot_Distanc,
												Constants.kGains_Distanc.kPeakOutput,
												Constants.kTimeoutMs);

		/* turn servo */
		_talonRght.Config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		_talonRght.Config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		_talonRght.Config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		_talonRght.Config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		_talonRght.Config_IntegralZone(Constants.kSlot_Turning, Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
		_talonRght.ConfigClosedLoopPeakOutput(	Constants.kSlot_Turning,
												Constants.kGains_Turning.kPeakOutput,
												Constants.kTimeoutMs);

		/* magic servo */
		_talonRght.Config_kP(Constants.kSlot_MotProf, Constants.kGains_MotProf.kP, Constants.kTimeoutMs);
		_talonRght.Config_kI(Constants.kSlot_MotProf, Constants.kGains_MotProf.kI, Constants.kTimeoutMs);
		_talonRght.Config_kD(Constants.kSlot_MotProf, Constants.kGains_MotProf.kD, Constants.kTimeoutMs);
		_talonRght.Config_kF(Constants.kSlot_MotProf, Constants.kGains_MotProf.kF, Constants.kTimeoutMs);
		_talonRght.Config_IntegralZone(Constants.kSlot_MotProf, Constants.kGains_MotProf.kIzone, Constants.kTimeoutMs);
		_talonRght.ConfigClosedLoopPeakOutput(	Constants.kSlot_MotProf,
												Constants.kGains_MotProf.kPeakOutput,
												Constants.kTimeoutMs);

		/* velocity servo */
		_talonRght.Config_kP(Constants.kSlot_Velocit, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		_talonRght.Config_kI(Constants.kSlot_Velocit, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		_talonRght.Config_kD(Constants.kSlot_Velocit, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		_talonRght.Config_kF(Constants.kSlot_Velocit, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		_talonRght.Config_IntegralZone(Constants.kSlot_Velocit, Constants.kGains_Velocit.kIzone, Constants.kTimeoutMs);
		_talonRght.ConfigClosedLoopPeakOutput(	Constants.kSlot_Velocit,
												Constants.kGains_Velocit.kPeakOutput,
												Constants.kTimeoutMs);

		_talonLeft.SetNeutralMode(NeutralMode::Brake);
		_talonRght.SetNeutralMode(NeutralMode::Brake);

		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_talonRght.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, closedLoopTimeMs, 0x00, Constants.PID_PRIMARY, Constants.kTimeoutMs);
		_talonRght.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, closedLoopTimeMs, 0x00, Constants.PID_TURN, Constants.kTimeoutMs);

		/**
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		_talonRght.ConfigAuxPIDPolarity(false, Constants.kTimeoutMs);

		ZeroSensors();
	}
	void ZeroSensors() {
		_talonLeft.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
		_talonRght.GetSensorCollection().SetQuadraturePosition(0, Constants.kTimeoutMs);
		_imu.SetYaw(0, Constants.kTimeoutMs);
		_imu.SetAccumZAngle(0, Constants.kTimeoutMs);
		printf("        [Sensors] All sensors are zeroed.\n");
	}
	void NeutralMotors(const char * reason) {
		_talonLeft.NeutralOutput();
		_talonRght.NeutralOutput();

		/* if caller is reporting WHY motors are being neutralized, report it */
		if (reason != nullptr) {
			printf("  Motors are neutral, ");
			printf(reason);
		}
	}
	void CommonLoop() {
		/* temps for the first calls or button events */
		bool bFirstCall = false;
		ButtonEvent bExecuteAction = ButtonEvent::ButtonOff;

		/* grab the joystick inputs */
		bool btns[Constants.kNumButtonsPlusOne];
		double joyFwd = -1 * _joy.GetY(); /* positive stick => forward */
		double joyTurn = +1 * _joy.GetTwist(); /* positive stick => right */
		double joyX = +1 * _joy.GetX();
		int pov = _joy.GetPOV();
		GetButtons(btns);

		/* dead-band the sticks */
		joyFwd = Deadband(joyFwd);
		joyTurn = Deadband(joyTurn);

		/* look for new POV (top-hat or D-pad.)
		 * https://en.wikipedia.org/wiki/D-pad */
		if (_pov != pov) {
			_pov = pov;
			switch (_pov) {
				case 270: /* 270 deg, D-PAD L */
					if (_selectedExample > 0) {
						/* move up to the next selected example type */
						_selectedExample = (ExampleType) (_selectedExample - 1);
					}
					break;
				case 90: /* 90 deg, D-PAD R */
					if ((_selectedExample + 1) < kTotalCount) {
						/* move up to the next selected example type */
						_selectedExample = (ExampleType) (_selectedExample + 1);
					}
					break;

				case 0: /* 0 deg, D-PAD Up */
				{ /* enable neutral-if-remote-LOSS-OF-SIGNAL */
					double value = 0;
					_talonRght.ConfigSetParameter(	ParamEnum::eRemoteSensorClosedLoopDisableNeutralOnLOS,
													value,
													0x00,
													0x00,
													Constants.kTimeoutMs);
					printf("Enable neutral if remote sensor is unplugged (Loss of Signal) \n");
				} break;

				case 180: /* 180 deg, D-PAD Down */
				{ /* disable neutral-if-remote-LOSS-OF-SIGNAL */
					double value = 1;
					_talonRght.ConfigSetParameter(	ParamEnum::eRemoteSensorClosedLoopDisableNeutralOnLOS,
													value,
													0x00,
													0x00,
													Constants.kTimeoutMs);
					printf("Disabled neutral if remote sensor is unplugged (Loss of Signal) \n");
				} break;
			}
		}

		/* look for new button presses */
		if (btns[6] && !_btns[6]) { /* if button6 is just pressed */
			/* take note that user pressed the action button */
			bExecuteAction = ButtonEvent::ButtonOffToOn;
		} else if (!btns[6] && _btns[6]) {
			bExecuteAction = ButtonEvent::ButtonOnToOff;
		} else if (btns[6]) {
			bExecuteAction = ButtonEvent::ButtonOn;
		} else {
			bExecuteAction = ButtonEvent::ButtonOff;
		}
		/* button 1*/
		if (btns[1] && !_btns[1]) {
			/* if button1 is just pressed */
			ZeroSensors();
		}
		CopyButtons(_btns, btns);

		/* user is holding shoulder button, execute selected example */
		if (_btns[5]) {
			/* button 5 is held down, jump to simple drive */
			if (_appliedExample != kOne_Axis_PercentOutput) {
				/* take note that this is the first loop of the new selection */
				bFirstCall = true;
				_appliedExample = kOne_Axis_PercentOutput;
			}
		} else if (_appliedExample != _selectedExample) {
			/* take note that this is the first loop of the new selection */
			bFirstCall = true;
			_appliedExample = _selectedExample;
		}

		switch (_appliedExample) {
			case kOne_Axis_PercentOutput:
				One_Axis_PercentOutput(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kTwo_Axis_PercentOutput:
				Two_Axis_PercentOutput(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kOne_Axis_Position:
				One_Axis_Position(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kOne_Axis_Position_WithCustomFeedFwd:
				One_Axis_Position_WithCustomFeedFwd(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kTwo_Axis_Position:
				Two_Axis_Position(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kOne_Axis_Velocity:
				One_Axis_Velocity(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kOne_Axis_Velocity_WithCustomFeedFwd:
				One_Axis_Velocity_WithCustomFeedFwd(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kTwo_Axis_Velocity:
				Two_Axis_Velocity(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kOne_Axis_MotionMagic:
				One_Axis_MotionMagic(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kOne_Axis_MotionMagic_WithCustomFeedFwd:
				One_Axis_MotionMagic_WithCustomFeedFwd(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kTwo_Axis_MotionMagic:
				Two_Axis_MotionMagic(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kOne_Axis_MotionProfile:
				One_Axis_MotionProfile(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kOne_Axis_MotionProfile_WithCustomFeedFwd:
				One_Axis_MotionProfile_WithCustomFeedFwd(bFirstCall, bExecuteAction, joyFwd, joyTurn, joyX);
				break;
			case kTwo_Axis_MotionProfile:
				Two_Axis_MotionProfile(bFirstCall, bExecuteAction, joyFwd, joyTurn);
				break;
			case kTwo_Axis_MotionProfile_WithCustomFeedFwd:
				Two_Axis_MotionProfile_WithCustomFeedFwd(bFirstCall, bExecuteAction, joyFwd, joyTurn, joyX);
				break;
			default:
				NeutralMotors("Invalid example selected\n");
				break;
		}
	}

//------------------------- Loops -------------------------------//
	void DisabledInit() {
		/* initialize hardware so that sensor phases on set before Teleop.
		 * This makes self-test more useful. */
		InitRobot();
	}
	void DisabledPeriodic() {
		CommonLoop();
	}
	void TeleopInit() {
		/* initialize hardware at start of teleop, just in case Talon was replaced / field-upgraded during disable.
		 * All params are persistent except for status frame periods. */
		InitRobot();
	}
	void TeleopPeriodic() {
		CommonLoop();
	}
	//------------------------- Examples -------------------------------//
	void One_Axis_PercentOutput(bool bFirstCall, ButtonEvent bExecuteAction, double joyY, double joyTurn) {

		/* calculate targets from gamepad inputs */
		double left = joyY + joyTurn;
		double rght = joyY - joyTurn;

		if (bFirstCall) {
			printf("[00]One_Axis_PercentOutput selected, ");
			printf("This is a basic arcade drove. \n");
		}

		_talonLeft.Set(ControlMode::PercentOutput, left);
		_talonRght.Set(ControlMode::PercentOutput, rght);
	}
	void Two_Axis_PercentOutput(bool bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

		/* calculate targets from gamepad inputs */
		(void) joyForward; /* not used so removed unused warning */
		(void) joyTurn; /* not used so removed unused warning */

		if (bFirstCall) {
			printf("[01]Two_Axis_PercentOutput selected, ");
			printf("This is a basic arcade drove. \n");
		}

		_talonLeft.Set(ControlMode::PercentOutput, joyForward, DemandType::DemandType_ArbitraryFeedForward, +joyTurn);
		_talonRght.Set(ControlMode::PercentOutput, joyForward, DemandType::DemandType_ArbitraryFeedForward, -joyTurn);
	}
	void One_Axis_Position(bool bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

		/* calculate targets from gamepad inputs */
		double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
		(void) joyTurn; /* not used so removed unused warning */

		if (bFirstCall) {
			printf("[02]One_Axis_Position selected, ");
			printf("Press Button 6 to set target. ");
			NeutralMotors("Target not set yet.\n");
			ZeroSensors();

			_talonRght.SelectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
		}

		if (bExecuteAction == ButtonEvent::ButtonOffToOn) {

		} else if (bExecuteAction == ButtonEvent::ButtonOn) {
			_target0 = target_sensorUnits;
			_talonRght.Set(ControlMode::Position, _target0);
			_talonLeft.Follow(_talonRght);
		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {
			//NeutralMotors("Button let go\n");
		}
	}
	void One_Axis_Position_WithCustomFeedFwd(	bool bFirstCall,
												ButtonEvent bExecuteAction,
												double joyForward,
												double joyTurn) {

		/* calculate targets from gamepad inputs */
		double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
		double feedFwdTerm = joyTurn * 0.25; /* how much motor output to add to the close loop output */

		if (bFirstCall) {
			printf("[03]One_Axis_Position_WithCustomFeedFwd selected, ");
			printf("Press Button 6 to set target. ");
			NeutralMotors("Target not set yet.\n");
			ZeroSensors();

			_talonRght.SelectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
		}

		if (bExecuteAction == ButtonEvent::ButtonOffToOn) {

		} else if (bExecuteAction == ButtonEvent::ButtonOn) {
			_target0 = target_sensorUnits;
			_target1 = feedFwdTerm;
			_talonRght.Set(ControlMode::Position, _target0, DemandType::DemandType_ArbitraryFeedForward, _target1);
			_talonLeft.Follow(_talonRght);
		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {
			//NeutralMotors("Button let go\n");
		}

	}

	void Two_Axis_Position(bool bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

		/* calculate targets from gamepad inputs */
		double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
		double target_turn = joyTurn * Constants.kTurnTravelUnitsPerRotation * -1.0; /* positive right stick => negative heading target (turn to right) */

		if (bFirstCall) {
			printf("[04]Two_Axis_Position selected, ");
			printf("Press Button 6 to set target. ");
			NeutralMotors("Target not set yet.\n");
			ZeroSensors();

			_talonRght.SelectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
			_talonRght.SelectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
		}

		if (bExecuteAction == ButtonEvent::ButtonOffToOn) {

		} else if (bExecuteAction == ButtonEvent::ButtonOn) {
			_target0 = target_sensorUnits;
			_target1 = target_turn;
			_talonRght.Set(ControlMode::Position, _target0, DemandType::DemandType_AuxPID, _target1);
			_talonLeft.Follow(_talonRght, FollowerType::FollowerType_AuxOutput1);
		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {
			//NeutralMotors("Button let go\n");
		}

	}

	void One_Axis_Velocity(bool bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

		/* calculate targets from gamepad inputs */
		double target_RPM = joyForward * 500; /* +- 500 RPM */
		double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;

		if (bFirstCall) {
			printf("[05]One_Axis_Velocity selected, ");
			printf("Press Button 6 to set target. ");
			NeutralMotors("Target not set yet.\n");

			_talonRght.SelectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
		}

		if (bExecuteAction == ButtonEvent::ButtonOffToOn) {

		} else if (bExecuteAction == ButtonEvent::ButtonOn) {
			_target0 = target_unitsPer100ms;
			_talonRght.Set(ControlMode::Velocity, _target0);
			_talonLeft.Follow(_talonRght);
		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {
			//NeutralMotors("Button let go\n");
		}

	}
	void One_Axis_Velocity_WithCustomFeedFwd(	bool bFirstCall,
												ButtonEvent bExecuteAction,
												double joyForward,
												double joyTurn) {

		/* calculate targets from gamepad inputs */
		double target_RPM = joyForward * 500; /* +- 500 RPM */
		double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;
		double feedFwdTerm = joyTurn * 0.25; /* how much to add to the close loop output */

		if (bFirstCall) {
			printf("[06]One_Axis_Velocity_WithCustomFeedFwd selected, ");
			printf("Press Button 6 to set target. ");
			NeutralMotors("Target not set yet.\n");

			_talonRght.SelectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
		}

		if (bExecuteAction == ButtonEvent::ButtonOffToOn) {

		} else if (bExecuteAction == ButtonEvent::ButtonOn) {
			_target0 = target_unitsPer100ms;
			_target1 = feedFwdTerm;
			_talonRght.Set(ControlMode::Velocity, _target0, DemandType::DemandType_ArbitraryFeedForward, _target1);
			_talonLeft.Follow(_talonRght);
		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {
			//NeutralMotors("Button let go\n");
		}

	}

	void Two_Axis_Velocity(bool bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

		/* calculate targets from gamepad inputs */
		double target_RPM = joyForward * 500; /* +- 500 RPM */
		double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;

		double heading_units = Constants.kTurnTravelUnitsPerRotation * joyTurn * -1.0; /* positive right stick => negative heading target (turn to right) */

		if (bFirstCall) {
			printf("[07]Two_Axis_Velocity selected, this can be used to approach a heading while mainting velocity. ");
			printf("Press Button 6 to set target. ");
			NeutralMotors("Target not set yet.\n");

			_talonRght.SelectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
			_talonRght.SelectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
		}

		if (bExecuteAction == ButtonEvent::ButtonOffToOn) {

		} else if (bExecuteAction == ButtonEvent::ButtonOn) {
			_target0 = target_unitsPer100ms;
			_target1 = heading_units;
			_talonRght.Set(ControlMode::Velocity, _target0, DemandType_AuxPID, _target1);
			_talonLeft.Follow(_talonRght, FollowerType::FollowerType_AuxOutput1);
		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {
			//NeutralMotors("Button let go\n");
		}

	}
	void One_Axis_MotionMagic(bool bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

		/* calculate targets from gamepad inputs */
		double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;

		if (bFirstCall) {
			printf("[08]One_Axis_MotionMagic selected, ");
			printf("Press Button 6 to set target. ");
			NeutralMotors("Target not set yet.\n");
			ZeroSensors();

			_talonRght.SelectProfileSlot(Constants.kSlot_MotProf, Constants.PID_PRIMARY);
		}

		if (bExecuteAction == ButtonEvent::ButtonOffToOn) {

		} else if (bExecuteAction == ButtonEvent::ButtonOn) {
			_target0 = target_sensorUnits;
			_talonRght.Set(ControlMode::MotionMagic, _target0);
			_talonLeft.Follow(_talonRght);
		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {
			//NeutralMotors("Button let go\n");
		}

	}
	void One_Axis_MotionMagic_WithCustomFeedFwd(bool bFirstCall,
												ButtonEvent bExecuteAction,
												double joyForward,
												double joyTurn) {

		/* calculate targets from gamepad inputs */
		double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
		double feedFwdTerm = joyTurn * 0.25;

		if (bFirstCall) {
			printf("[09]One_Axis_MotionMagic_WithCustomFeedFwd selected, ");
			printf("Press Button 6 to set target. ");
			NeutralMotors("Target not set yet.\n");
			ZeroSensors();

			_talonRght.SelectProfileSlot(Constants.kSlot_MotProf, Constants.PID_PRIMARY);
		}

		if (bExecuteAction == ButtonEvent::ButtonOffToOn) {

		} else if (bExecuteAction == ButtonEvent::ButtonOn) {
			_target0 = target_sensorUnits;
			_target1 = feedFwdTerm;
			_talonRght.Set(ControlMode::MotionMagic, _target0, DemandType::DemandType_ArbitraryFeedForward, _target1);
			_talonLeft.Follow(_talonRght);
		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {
			//NeutralMotors("Button let go\n");
		}

	}
	void Two_Axis_MotionMagic(bool bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

		/* calculate targets from gamepad inputs */
		double target_sensorUnits = joyForward * Constants.kSensorUnitsPerRotation * Constants.kRotationsToTravel;
		double heading_units = joyTurn * Constants.kTurnTravelUnitsPerRotation * -1.0; /* positive right stick => negative heading target (turn to right) */

		if (bFirstCall) {
			printf("[10]Two_Axis_MotionMagic selected, ");
			printf("Press Button 6 to set target. ");
			NeutralMotors("Target not set yet.\n");
			ZeroSensors();
			_talonRght.SelectProfileSlot(Constants.kSlot_MotProf, Constants.PID_PRIMARY);
			_talonRght.SelectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
		}

		if (bExecuteAction == ButtonEvent::ButtonOffToOn) {

		} else if (bExecuteAction == ButtonEvent::ButtonOn) {
			_target0 = target_sensorUnits;
			_target1 = heading_units;

			_talonRght.Set(ControlMode::MotionMagic, _target0, DemandType_AuxPID, _target1);
			_talonLeft.Follow(_talonRght, FollowerType::FollowerType_AuxOutput1);
		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {
			//NeutralMotors("Button let go\n");
		}
	}
	void One_Axis_MotionProfile(bool bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

		/* calculate targets from gamepad inputs */
		bool bMoveForward = (joyForward >= 0) ? true : false;
		(void) joyTurn; /* not used, so remove the unused warning */

		if (bFirstCall) {
			printf("[11]One_Axis_MotionProfile selected, ");
			printf("Press Button 6 to fire the profile. ");
			NeutralMotors("Target not set yet.\n");

			/* slots are selected in the profile, not via SelectProfileSlot() */

		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {


		} else if (bExecuteAction == ButtonEvent::ButtonOffToOn) {

			NeutralMotors("Button let go\n");
			ZeroSensors();
			_motProfExample->reset();
			_motProfExample->start(0, bMoveForward); /*final target heading is ignored, doesn't matter */


		} else if (bExecuteAction == ButtonEvent::ButtonOn) {

			_talonRght.Set(ControlMode::MotionProfile, _motProfExample->getSetValue());
			_talonLeft.Follow(_talonRght);
		}

		/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
		_motProfExample->control();
		_motProfExample->PeriodicTask();
	}
	void One_Axis_MotionProfile_WithCustomFeedFwd(	bool bFirstCall,
													ButtonEvent bExecuteAction,
													double joyForward,
													double joyTurn,
													double joyX) {
		/* calculate targets from gamepad inputs */
		double feedFwdTerm = 0.25 * joyX;
		bool bMoveForward = (joyForward >= 0) ? true : false;
		(void) joyTurn; /* not used, so remove the unused warning */

		if (bFirstCall) {
			printf("[12]One_Axis_MotionProfile_WithCustomFeedFwd selected, ");
			printf("Press Button 6 to fire the profile. ");
			NeutralMotors("Target not set yet.\n");

			/* slots are selected in the profile, not via SelectProfileSlot() */

		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {

		} else if (bExecuteAction == ButtonEvent::ButtonOffToOn) {
			NeutralMotors("Button let go\n");
			ZeroSensors();
			_motProfExample->reset();
			_motProfExample->start(0, bMoveForward); /*final target heading is ignored, doesn't matter */
		} else if (bExecuteAction == ButtonEvent::ButtonOn) {

			_talonRght.Set(	ControlMode::MotionProfile,
							_motProfExample->getSetValue(),
							DemandType::DemandType_ArbitraryFeedForward,
							feedFwdTerm);
			_talonLeft.Follow(_talonRght);
		}
		/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
		_motProfExample->control();
		_motProfExample->PeriodicTask();

	}
	void Two_Axis_MotionProfile(bool bFirstCall, ButtonEvent bExecuteAction, double joyForward, double joyTurn) {

		/* calculate targets from gamepad inputs */
		bool bMoveForward = (joyForward >= 0) ? true : false;
		double finalHeading_units = Constants.kTurnTravelUnitsPerRotation * joyTurn * -1.0; /* positive right stick => negative heading target (turn to right) */

		if (bFirstCall) {
			printf("[13]Two_Axis_MotionProfile selected, ");
			printf("Press Button 6 to fire the profile. ");
			NeutralMotors("Target not set yet.\n");

			/* slots are selected in the profile, not via SelectProfileSlot() */

		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {

		} else if (bExecuteAction == ButtonEvent::ButtonOffToOn) {
			NeutralMotors("Button let go\n");
			ZeroSensors();
			_motProfExample->reset();
			_motProfExample->start(finalHeading_units, bMoveForward);
		} else if (bExecuteAction == ButtonEvent::ButtonOn) {

			_talonRght.Set(ControlMode::MotionProfileArc, _motProfExample->getSetValue());
			_talonLeft.Follow(_talonRght, FollowerType::FollowerType_AuxOutput1);
		}
		/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
		_motProfExample->control();
		_motProfExample->PeriodicTask();

	}
	void Two_Axis_MotionProfile_WithCustomFeedFwd(	bool bFirstCall,
													ButtonEvent bExecuteAction,
													double joyForward,
													double joyTurn,
													double joyX) {
		/* calculate targets from gamepad inputs */
		double feedFwdTerm = 0.25 * joyX;
		bool bMoveForward = (joyForward >= 0) ? true : false;
		double finalHeading_units = Constants.kTurnTravelUnitsPerRotation * joyTurn * -1.0; /* positive right stick => negative heading target (turn to right) */

		if (bFirstCall) {
			printf("[14]Two_Axis_MotionProfile_WithCustomFeedFwd selected, ");
			printf("Press Button 6 to fire the profile. ");
			NeutralMotors("Target not set yet.\n");

			/* slots are selected in the profile, not via SelectProfileSlot() */

		} else if (bExecuteAction == ButtonEvent::ButtonOnToOff) {

		} else if (bExecuteAction == ButtonEvent::ButtonOffToOn) {
			NeutralMotors("Button let go\n");
			ZeroSensors();
			_motProfExample->reset();
			_motProfExample->start(finalHeading_units, bMoveForward);
		} else if (bExecuteAction == ButtonEvent::ButtonOn) {
			_talonRght.Set(	ControlMode::MotionProfileArc,
							_motProfExample->getSetValue(),
							DemandType::DemandType_ArbitraryFeedForward,
							feedFwdTerm);
			_talonLeft.Follow(_talonRght, FollowerType::FollowerType_AuxOutput1);
		}

		/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
		_motProfExample->control();
		_motProfExample->PeriodicTask();
	}
	//-------------- Some helpful routines ---------------//
	void GetButtons(bool * btns) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			btns[i] = _joy.GetRawButton(i);
		}
	}
	void CopyButtons(bool * destination, const bool * source) {
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
};

START_ROBOT_CLASS(Robot)
