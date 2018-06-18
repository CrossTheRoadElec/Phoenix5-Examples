package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.CANifierConfiguration;
import com.ctre.phoenix.sensors.PigeonIMUConfiguration;;


public class Configs {

    public TalonSRXConfiguration _talon;
    public VictorSPXConfiguration _victor;
    public PigeonIMUConfiguration _pigeon;
    public CANifierConfiguration _canifier;
    
    
    public Configs()
    {
        /*Construct all of the configurations with any set of values
         *These are just arbitrary values to demonstrate the feature
         */
        _talon = new TalonSRXConfiguration();
        _victor = new VictorSPXConfiguration();
        _pigeon = new PigeonIMUConfiguration();
        _canifier = new CANifierConfiguration();
    
    
    
        //TalonSRX:
        _talon.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        _talon.primaryPID.selectedFeedbackCoefficient = 0.328293;
        _talon.auxilaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;
        _talon.auxilaryPID.selectedFeedbackCoefficient = 0.877686;
        _talon.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
        _talon.reverseLimitSwitchSource = LimitSwitchSource.RemoteTalonSRX;
        _talon.sum_0 = FeedbackDevice.QuadEncoder;
        _talon.sum_1 = FeedbackDevice.RemoteSensor0;
        _talon.diff_0 = FeedbackDevice.RemoteSensor1;
        _talon.diff_1 = FeedbackDevice.PulseWidthEncodedPosition;
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
        _talon.velocityMeasurementPeriod = VelocityMeasPeriod.Period_25Ms;
        _talon.velocityMeasurementWindow = 8;
        _talon.forwardLimitSwitchDeviceID = 6;
        _talon.reverseLimitSwitchDeviceID = 5;
        _talon.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;
        _talon.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
        _talon.forwardSoftLimitThreshold = 2767;
        _talon.reverseSoftLimitThreshold = -1219;
        _talon.forwardSoftLimitEnable = true;
        _talon.reverseSoftLimitEnable = true;
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
        _talon.auxPIDPolarity = true;
        _talon.filter_0.remoteSensorDeviceID = 22;
        _talon.filter_0.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Roll;
        _talon.filter_1.remoteSensorDeviceID = 41;
        _talon.filter_1.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Yaw;
        _talon.motionCruiseVelocity = 37;
        _talon.motionAcceleration = 3;
        _talon.motionProfileTrajectoryPeriod = 11;
        _talon.feedbackNotContinuous = true;
        _talon.remoteSensorClosedLoopDisableNeutralOnLOS = false;
        _talon.clearPositionOnLimitF = true;
        _talon.clearPositionOnLimitR = true;
        _talon.clearPositionOnQuadIdx = false;
        _talon.limitSwitchDisableNeutralOnLOS = true;
        _talon.softLimitDisableNeutralOnLOS = false;
        _talon.pulseWidthPeriod_EdgesPerRot = 9;
        _talon.pulseWidthPeriod_FilterWindowSz = 32;
        _talon.customParam_0 = 3;
        _talon.customParam_1 = 5;
    
        //VictorSPX: 
        _victor.primaryPID.selectedFeedbackSensor = RemoteFeedbackDevice.SoftwareEmulatedSensor;
        _victor.primaryPID.selectedFeedbackCoefficient = 0.122208;
        _victor.auxilaryPID.selectedFeedbackSensor = RemoteFeedbackDevice.SensorDifference;
        _victor.auxilaryPID.selectedFeedbackCoefficient = 0.290985;
        _victor.forwardLimitSwitchSource = RemoteLimitSwitchSource.RemoteTalonSRX;
        _victor.reverseLimitSwitchSource = RemoteLimitSwitchSource.Deactivated;
        _victor.sum_0 = RemoteFeedbackDevice.RemoteSensor0;
        _victor.sum_1 = RemoteFeedbackDevice.RemoteSensor1;
        _victor.diff_0 = RemoteFeedbackDevice.SoftwareEmulatedSensor;
        _victor.diff_1 = RemoteFeedbackDevice.RemoteSensor0;
        _victor.openloopRamp = 0.300882;
        _victor.closedloopRamp = 0.100294;
        _victor.peakOutputForward = 0.659824;
        _victor.peakOutputReverse = -0.769306;
        _victor.nominalOutputForward = 0.099707;
        _victor.nominalOutputReverse = -0.119257;
        _victor.neutralDeadband = 0.049853;
        _victor.voltageCompSaturation = 10.699219;
        _victor.voltageMeasurementFilter = 32;
        _victor.velocityMeasurementPeriod = VelocityMeasPeriod.Period_50Ms;
        _victor.velocityMeasurementWindow = 4;
        _victor.forwardLimitSwitchDeviceID = 8;
        _victor.reverseLimitSwitchDeviceID = 7;
        _victor.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
        _victor.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;
        _victor.forwardSoftLimitThreshold = 2326;
        _victor.reverseSoftLimitThreshold = -9023;
        _victor.forwardSoftLimitEnable = false;
        _victor.reverseSoftLimitEnable = true;
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
        _victor.auxPIDPolarity = true;
        _victor.filter_0.remoteSensorDeviceID = 22;
        _victor.filter_0.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Roll;
        _victor.filter_1.remoteSensorDeviceID = 41;
        _victor.filter_1.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Yaw;
        _victor.motionCruiseVelocity = 50;
        _victor.motionAcceleration = 3;
        _victor.motionProfileTrajectoryPeriod = 20;
        _victor.feedbackNotContinuous = false;
        _victor.remoteSensorClosedLoopDisableNeutralOnLOS = true;
        _victor.clearPositionOnLimitF = false;
        _victor.clearPositionOnLimitR = false;
        _victor.clearPositionOnQuadIdx = true;
        _victor.limitSwitchDisableNeutralOnLOS = false;
        _victor.softLimitDisableNeutralOnLOS = true;
        _victor.pulseWidthPeriod_EdgesPerRot = 8;
        _victor.pulseWidthPeriod_FilterWindowSz = 19;
        _victor.customParam_0 = 7;
        _victor.customParam_1 = 9;
    
    
        //PigeonIMU:
        _pigeon.temperatureCompensationDisable = true;
        _pigeon.customParam_0 = 6;
        _pigeon.customParam_1 = 14;
    
        //CANifier
        _canifier.velocityMeasurementPeriod = VelocityPeriod.Period_50Ms;
        _canifier.velocityMeasurementWindow = 8;
        _canifier.customParam_0 = 2;
        _canifier.customParam_1 = 1;
    }
}
