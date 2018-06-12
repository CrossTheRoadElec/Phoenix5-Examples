/*
 * Configs.h
 *
 *  Created on: Jun 8, 2018
 *      Author: Ryan Greenblatt
 */

#ifndef SRC_CONFIGS_H_
#define SRC_CONFIGS_H_

#include "ctre/Phoenix.h"

struct configs {

	ctre::phoenix::motorcontrol::can::TalonSRXConfiguration _talon;
	ctre::phoenix::motorcontrol::can::VictorSPXConfiguration _victor;
	ctre::phoenix::sensors::PigeonIMUConfiguration pigeon_;
	ctre::phoenix::CANifierConfiguration canifier_;

	
	configs() {
		/*Construct all of the configurations with any set of values
		 *These are just arbitrary values to demonstrate the feature
		 */

		//TalonSRX:
		_talon.openloopRamp = 1.0; 
		_talon.closedloopRamp = 1.5;
		_talon.peakOutputForward = .94; 
		_talon.peakOutputReverse = -.29; 
		_talon.nominalOutputForward = .74; 
		_talon.nominalOutputReverse = -.12; 
		_talon.neutralDeadband = 0.2; 
		
		_talon.voltageCompSaturation = 9.3;
		_talon.voltageMeasurementFilter = 16;
		
		_talon.velocityMeasurementPeriod = ctre::phoenix::motorcontrol::Period_25Ms;
		_talon.velocityMeasurementWindow = 8;
		
		_talon.forwardLimitSwitchDeviceID = 6; //TODO
		_talon.reverseLimitSwitchDeviceID = 5;
		_talon.forwardLimitSwitchNormal = LimitSwitchNormal_Disabled;
		_talon.reverseLimitSwitchNormal = LimitSwitchNormal_NormallyClosed;
		
		_talon.forwardSoftLimitThreshold = 2767;
		_talon.reverseSoftLimitThreshold = -1219;
		_talon.forwardSoftLimitEnable = true;
		_talon.reverseSoftLimitEnable = true;
		
		_talon.slot_0.kP = 504;
		_talon.slot_0.kI = 5.6;
		_talon.slot_0.kD = 0.2;
		_talon.slot_0.kF = 19.3; 
		_talon.slot_0.integralZone = 900; 
		_talon.slot_0.allowableClosedloopError = 217; 
		_talon.slot_0.maxIntegralAccumulator = 254;
		_talon.slot_0.closedLoopPeakOutput = .87;
		_talon.slot_0.closedLoopPeriod = 33; 

		_talon.slot_1.kP = 155.6;
		_talon.slot_1.kI = 5.56;
		_talon.slot_1.kD = 8.8686;
		_talon.slot_1.kF = 454.0; 
		_talon.slot_1.integralZone = 100; 
		_talon.slot_1.allowableClosedloopError = 200; 
		_talon.slot_1.maxIntegralAccumulator = 91;
		_talon.slot_1.closedLoopPeakOutput = .2;
		_talon.slot_1.closedLoopPeriod = 34;

		_talon.slot_2.kP = 2.23232e2;
		_talon.slot_2.kI = 34;
		_talon.slot_2.kD = 67;
		_talon.slot_2.kF = 2*3+0.323232323; 
		_talon.slot_2.integralZone = 44; 
		_talon.slot_2.allowableClosedloopError = 343; 
		_talon.slot_2.maxIntegralAccumulator = 334;
		_talon.slot_2.closedLoopPeakOutput = .4;
		_talon.slot_2.closedLoopPeriod = 14;

		_talon.slot_3.kP = 34;
		_talon.slot_3.kI = 32;
		_talon.slot_3.kD = 436;
		_talon.slot_3.kF = .34343; 
		_talon.slot_3.integralZone = 2323; 
		_talon.slot_3.allowableClosedloopError = 543; 
		_talon.slot_3.maxIntegralAccumulator = 687;
		_talon.slot_3.closedLoopPeakOutput = .13;
		_talon.slot_3.closedLoopPeriod = 12;

		_talon.auxPIDPolarity = true;
		_talon.motionCruiseVelocity = 37;
		_talon.motionAcceleration = 3;
		_talon.motionProfileTrajectoryPeriod = 11;

		_talon.primaryPID.selectedFeedbackCoefficient = .3283;
		_talon.primaryPID.selectedFeedbackSensor = RemoteSensor0;	
		
		_talon.auxilaryPID.selectedFeedbackCoefficient = .8777;
		_talon.auxilaryPID.selectedFeedbackSensor = Analog;	

		_talon.filter_0.remoteSensorDeviceID = 22;
		_talon.filter_0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll;

		_talon.filter_1.remoteSensorDeviceID = 41;
		_talon.filter_1.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw; 
		
        _talon.sum_0 = QuadEncoder; 
		_talon.sum_1 = RemoteSensor0; 
        _talon.diff_0 = RemoteSensor1; 
		_talon.diff_1 = PulseWidthEncodedPosition; 
		
        _talon.forwardLimitSwitchSource = LimitSwitchSource_Deactivated;
		_talon.reverseLimitSwitchSource = LimitSwitchSource_RemoteTalonSRX;
		_talon.peakCurrentLimit = 20;          
		_talon.peakCurrentDuration = 200;	   
		_talon.continuousCurrentLimit = 30;    
	
		//VictorSPX: 
		_victor.openloopRamp = 0.3;
		_victor.closedloopRamp = 0.1;
		_victor.peakOutputForward = .66;
		_victor.peakOutputReverse = -.77;
		_victor.nominalOutputForward = .1;
		_victor.nominalOutputReverse = -.12;
		_victor.neutralDeadband = .05;
		
		_victor.voltageCompSaturation = 10.7;
		_victor.voltageMeasurementFilter = 32;
		
		_victor.velocityMeasurementPeriod = ctre::phoenix::motorcontrol::Period_50Ms;
		_victor.velocityMeasurementWindow = 4;
		
		_victor.forwardLimitSwitchDeviceID = 8;
		_victor.reverseLimitSwitchDeviceID = 7;
		_victor.forwardLimitSwitchNormal = LimitSwitchNormal_NormallyClosed;
		_victor.reverseLimitSwitchNormal = LimitSwitchNormal_Disabled;
		
		_victor.forwardSoftLimitThreshold = 2326;
		_victor.reverseSoftLimitThreshold = -9023;
		_victor.forwardSoftLimitEnable = false;
		_victor.reverseSoftLimitEnable = true;
		
		_victor.slot_0.kP = .1;
		_victor.slot_0.kI = .01;
		_victor.slot_0.kD = .05;
		_victor.slot_0.kF = .32; 
		_victor.slot_0.integralZone = 900; 
		_victor.slot_0.allowableClosedloopError = 124; 
		_victor.slot_0.maxIntegralAccumulator = 22;
		_victor.slot_0.closedLoopPeakOutput = .6;
		_victor.slot_0.closedLoopPeriod = 33;

		_victor.slot_1.kP = .4;
		_victor.slot_1.kI = .3;
		_victor.slot_1.kD = .1;
		_victor.slot_1.kF = .923; 
		_victor.slot_1.integralZone = 90; 
		_victor.slot_1.allowableClosedloopError = 21; 
		_victor.slot_1.maxIntegralAccumulator = 54;
		_victor.slot_1.closedLoopPeakOutput = .4;
		_victor.slot_1.closedLoopPeriod = 23;

		_victor.slot_2.kP = 2.2;
		_victor.slot_2.kI = 0.67;
		_victor.slot_2.kD = 9.232;
		_victor.slot_2.kF = 1.121; 
		_victor.slot_2.integralZone = 19; 
		_victor.slot_2.allowableClosedloopError = 190; 
		_victor.slot_2.maxIntegralAccumulator = 37;
		_victor.slot_2.closedLoopPeakOutput = 0.3;
		_victor.slot_2.closedLoopPeriod = 9;

		_victor.slot_3.kP = 10;
		_victor.slot_3.kI = 11;
		_victor.slot_3.kD = 12;
		_victor.slot_3.kF = 13; 
		_victor.slot_3.integralZone = 654; 
		_victor.slot_3.allowableClosedloopError = 557; 
		_victor.slot_3.maxIntegralAccumulator = 342;
		_victor.slot_3.closedLoopPeakOutput = 0.9;
		_victor.slot_3.closedLoopPeriod = 21;

		_victor.auxPIDPolarity = true;
		_victor.motionCruiseVelocity = 50;
		_victor.motionAcceleration = 3;
		_victor.motionProfileTrajectoryPeriod = 20;

		_victor.primaryPID.selectedFeedbackCoefficient = .12222;
		_victor.primaryPID.selectedFeedbackSensor = RemoteFeedbackDevice::RemoteFeedbackDevice_SoftwareEmulatedSensor;

		_victor.auxilaryPID.selectedFeedbackCoefficient = .291;
		_victor.auxilaryPID.selectedFeedbackSensor = RemoteFeedbackDevice::RemoteFeedbackDevice_SensorDifference;
		
        _victor.filter_0.remoteSensorDeviceID = 22;
		_victor.filter_0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll;

		_victor.filter_1.remoteSensorDeviceID = 41;
		_victor.filter_1.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw; 
		
        _victor.sum_0 = RemoteFeedbackDevice_RemoteSensor0; //TODO
		_victor.sum_1 = RemoteFeedbackDevice_RemoteSensor1; //TODO
        _victor.diff_0 = RemoteFeedbackDevice_SoftwareEmulatedSensor; //TODO
		_victor.diff_1 = RemoteFeedbackDevice_RemoteSensor0; //TODO
		
		_victor.forwardLimitSwitchSource = RemoteLimitSwitchSource_RemoteTalonSRX;
		_victor.reverseLimitSwitchSource = RemoteLimitSwitchSource_Deactivated;

	}
};



#endif /* SRC_CONFIGS_H_ */
