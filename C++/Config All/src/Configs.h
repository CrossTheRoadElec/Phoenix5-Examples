#ifndef SRC_CONFIGS_H_
#define SRC_CONFIGS_H_

#include "ctre/Phoenix.h"

struct configs {

    /*Hold all of the config data*/
	ctre::phoenix::motorcontrol::can::TalonSRXConfiguration _talon;
	ctre::phoenix::motorcontrol::can::VictorSPXConfiguration _victor;
	ctre::phoenix::sensors::PigeonIMUConfiguration _pigeon;
	ctre::phoenix::CANifierConfiguration _canifier;

	
	configs() {
		/*Construct all of the configurations with any set of values
		 *These are just arbitrary values to demonstrate the feature
         */

		//TalonSRX:
        _talon.primaryPID.selectedFeedbackSensor = None;
        _talon.primaryPID.selectedFeedbackCoefficient = 0.328293;
        _talon.auxilaryPID.selectedFeedbackSensor = Analog;
        _talon.auxilaryPID.selectedFeedbackCoefficient = 0.877686;
        _talon.forwardLimitSwitchSource = LimitSwitchSource_Deactivated;
        _talon.reverseLimitSwitchSource = LimitSwitchSource_RemoteTalonSRX;
        _talon.sum_0 = QuadEncoder;
        _talon.sum_1 = RemoteSensor0;
        _talon.diff_0 = RemoteSensor1;
        _talon.diff_1 = PulseWidthEncodedPosition;
        _talon.peakCurrentLimit = 20;
        _talon.peakCurrentDuration = 200;
        _talon.continuousCurrentLimit = 30;
        _talon.openloopRamp = 1.023000;
        _talon.closedloopRamp = 1.705000;
        _talon.peakOutputForward = 0.939394;
        _talon.peakOutputReverse = -0.289345;
        _talon.nominalOutputForward = 0.739980;
        _talon.nominalOutputReverse = -0.119257;
        _talon.neutralDeadband = 0.199413;
        _talon.voltageCompSaturation = 9.296875;
        _talon.voltageMeasurementFilter = 16;
        _talon.velocityMeasurementPeriod = VelocityMeasPeriod::Period_25Ms;
        _talon.velocityMeasurementWindow = 8;
        _talon.forwardLimitSwitchDeviceID = 6;
        _talon.reverseLimitSwitchDeviceID = 5;
        _talon.forwardLimitSwitchNormal = LimitSwitchNormal_Disabled;
        _talon.reverseLimitSwitchNormal = LimitSwitchNormal_NormallyClosed;
        _talon.forwardSoftLimitThreshold = 2767;
        _talon.reverseSoftLimitThreshold = -1219;
        _talon.forwardSoftLimitEnable = 1;
        _talon.reverseSoftLimitEnable = 1;
        _talon.slot_0.kP = 504.000000;
        _talon.slot_0.kI = 5.600000;
        _talon.slot_0.kD = 0.200000;
        _talon.slot_0.kF = 19.300000;
        _talon.slot_0.integralZone = 900;
        _talon.slot_0.allowableClosedloopError = 217;
        _talon.slot_0.maxIntegralAccumulator = 254.000000;
        _talon.slot_0.closedLoopPeakOutput = 0.869990;
        _talon.slot_0.closedLoopPeriod = 33;
        _talon.slot_1.kP = 155.600000;
        _talon.slot_1.kI = 5.560000;
        _talon.slot_1.kD = 8.868600;
        _talon.slot_1.kF = 454.000000;
        _talon.slot_1.integralZone = 100;
        _talon.slot_1.allowableClosedloopError = 200;
        _talon.slot_1.maxIntegralAccumulator = 91.000000;
        _talon.slot_1.closedLoopPeakOutput = 0.199413;
        _talon.slot_1.closedLoopPeriod = 34;
        _talon.slot_2.kP = 223.232000;
        _talon.slot_2.kI = 34.000000;
        _talon.slot_2.kD = 67.000000;
        _talon.slot_2.kF = 6.323232;
        _talon.slot_2.integralZone = 44;
        _talon.slot_2.allowableClosedloopError = 343;
        _talon.slot_2.maxIntegralAccumulator = 334.000000;
        _talon.slot_2.closedLoopPeakOutput = 0.399804;
        _talon.slot_2.closedLoopPeriod = 14;
        _talon.slot_3.kP = 34.000000;
        _talon.slot_3.kI = 32.000000;
        _talon.slot_3.kD = 436.000000;
        _talon.slot_3.kF = 0.343430;
        _talon.slot_3.integralZone = 2323;
        _talon.slot_3.allowableClosedloopError = 543;
        _talon.slot_3.maxIntegralAccumulator = 687.000000;
        _talon.slot_3.closedLoopPeakOutput = 0.129032;
        _talon.slot_3.closedLoopPeriod = 12;
        _talon.auxPIDPolarity = 1;
        _talon.filter_0.remoteSensorDeviceID = 22;
        _talon.filter_0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll;
        _talon.filter_1.remoteSensorDeviceID = 41;
        _talon.filter_1.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw;
        _talon.motionCruiseVelocity = 37;
        _talon.motionAcceleration = 3;
        _talon.motionProfileTrajectoryPeriod = 11;
        _talon.feedbackNotContinuous = 1;
        _talon.remoteSensorClosedLoopDisableNeutralOnLOS = 0;
        _talon.clearPositionOnLimitF = 1;
        _talon.clearPositionOnLimitR = 1;
        _talon.clearPositionOnQuadIdx = 0;
        _talon.limitSwitchDisableNeutralOnLOS = 1;
        _talon.softLimitDisableNeutralOnLOS = 0;
        _talon.pulseWidthPeriod_EdgesPerRot = 9;
        _talon.pulseWidthPeriod_FilterWindowSz = 32;
        _talon.customParam_0 = 3;
        _talon.customParam_1 = 5;

		//VictorSPX: 
        _victor.primaryPID.selectedFeedbackSensor = RemoteFeedbackDevice_SoftwareEmulatedSensor;
        _victor.primaryPID.selectedFeedbackCoefficient = 0.122208;
        _victor.auxilaryPID.selectedFeedbackSensor = RemoteFeedbackDevice_SensorDifference;
        _victor.auxilaryPID.selectedFeedbackCoefficient = 0.290985;
        _victor.forwardLimitSwitchSource = RemoteLimitSwitchSource_RemoteTalonSRX;
        _victor.reverseLimitSwitchSource = RemoteLimitSwitchSource_Deactivated;
        _victor.sum_0 = RemoteFeedbackDevice_RemoteSensor0;
        _victor.sum_1 = RemoteFeedbackDevice_RemoteSensor1;
        _victor.diff_0 = RemoteFeedbackDevice_SoftwareEmulatedSensor;
        _victor.diff_1 = RemoteFeedbackDevice_RemoteSensor0;
        _victor.openloopRamp = 0.300882;
        _victor.closedloopRamp = 0.100294;
        _victor.peakOutputForward = 0.659824;
        _victor.peakOutputReverse = -0.769306;
        _victor.nominalOutputForward = 0.099707;
        _victor.nominalOutputReverse = -0.119257;
        _victor.neutralDeadband = 0.049853;
        _victor.voltageCompSaturation = 10.699219;
        _victor.voltageMeasurementFilter = 32;
        _victor.velocityMeasurementPeriod = VelocityMeasPeriod::Period_50Ms;
        _victor.velocityMeasurementWindow = 4;
        _victor.forwardLimitSwitchDeviceID = 8;
        _victor.reverseLimitSwitchDeviceID = 7;
        _victor.forwardLimitSwitchNormal = LimitSwitchNormal_NormallyClosed;
        _victor.reverseLimitSwitchNormal = LimitSwitchNormal_Disabled;
        _victor.forwardSoftLimitThreshold = 2326;
        _victor.reverseSoftLimitThreshold = -9023;
        _victor.forwardSoftLimitEnable = 0;
        _victor.reverseSoftLimitEnable = 1;
        _victor.slot_0.kP = 0.100000;
        _victor.slot_0.kI = 0.010000;
        _victor.slot_0.kD = 0.050000;
        _victor.slot_0.kF = 0.320000;
        _victor.slot_0.integralZone = 900;
        _victor.slot_0.allowableClosedloopError = 124;
        _victor.slot_0.maxIntegralAccumulator = 22.000000;
        _victor.slot_0.closedLoopPeakOutput = 0.599218;
        _victor.slot_0.closedLoopPeriod = 33;
        _victor.slot_1.kP = 0.400000;
        _victor.slot_1.kI = 0.300000;
        _victor.slot_1.kD = 0.100000;
        _victor.slot_1.kF = 0.923000;
        _victor.slot_1.integralZone = 90;
        _victor.slot_1.allowableClosedloopError = 21;
        _victor.slot_1.maxIntegralAccumulator = 54.000000;
        _victor.slot_1.closedLoopPeakOutput = 0.399804;
        _victor.slot_1.closedLoopPeriod = 23;
        _victor.slot_2.kP = 2.200000;
        _victor.slot_2.kI = 0.670000;
        _victor.slot_2.kD = 9.232000;
        _victor.slot_2.kF = 1.121000;
        _victor.slot_2.integralZone = 19;
        _victor.slot_2.allowableClosedloopError = 190;
        _victor.slot_2.maxIntegralAccumulator = 37.000000;
        _victor.slot_2.closedLoopPeakOutput = 0.299120;
        _victor.slot_2.closedLoopPeriod = 9;
        _victor.slot_3.kP = 10.000000;
        _victor.slot_3.kI = 11.000000;
        _victor.slot_3.kD = 12.000000;
        _victor.slot_3.kF = 13.000000;
        _victor.slot_3.integralZone = 654;
        _victor.slot_3.allowableClosedloopError = 557;
        _victor.slot_3.maxIntegralAccumulator = 342.000000;
        _victor.slot_3.closedLoopPeakOutput = 0.899316;
        _victor.slot_3.closedLoopPeriod = 21;
        _victor.auxPIDPolarity = 1;
        _victor.filter_0.remoteSensorDeviceID = 22;
        _victor.filter_0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll;
        _victor.filter_1.remoteSensorDeviceID = 41;
        _victor.filter_1.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw;
        _victor.motionCruiseVelocity = 50;
        _victor.motionAcceleration = 3;
        _victor.motionProfileTrajectoryPeriod = 20;
        _victor.feedbackNotContinuous = 0;
        _victor.remoteSensorClosedLoopDisableNeutralOnLOS = 1;
        _victor.clearPositionOnLimitF = 0;
        _victor.clearPositionOnLimitR = 0;
        _victor.clearPositionOnQuadIdx = 1;
        _victor.limitSwitchDisableNeutralOnLOS = 0;
        _victor.softLimitDisableNeutralOnLOS = 1;
        _victor.pulseWidthPeriod_EdgesPerRot = 8;
        _victor.pulseWidthPeriod_FilterWindowSz = 19;
        _victor.customParam_0 = 7;
        _victor.customParam_1 = 9;


        //PigeonIMU:
        _pigeon.temperatureCompensationDisable = false;
        _pigeon.customParam_0 = 6;
        _pigeon.customParam_1 = 14;

        //CANifier
        _canifier.velocityMeasurementPeriod = ctre::phoenix::Period_50Ms;
        _canifier.velocityMeasurementWindow = 8;
		_canifier.clearPositionOnLimitF = true;
		_canifier.clearPositionOnLimitR = false;
		_canifier.clearPositionOnQuadIdx = true;
        _canifier.customParam_0 = 2;
        _canifier.customParam_1 = 1;

	}
};



#endif /* SRC_CONFIGS_H_ */
