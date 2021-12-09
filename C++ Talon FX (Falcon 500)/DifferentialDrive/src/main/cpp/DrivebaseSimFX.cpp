#include "DrivebaseSimFX.h"
#include <wpi/numbers>

/**
 * Creates a new drivebase simualtor using Falcon 500 motors.
 * 
 * @param leftMaster the left master Falcon
 * @param rightMaster the right master Falcon
 * @param pidgey the Pigeon IMU
 */
DrivebaseSimFX::DrivebaseSimFX(WPI_TalonFX& leftMaster, WPI_TalonFX& rightMaster, WPI_PigeonIMU& pidgey)
	: _leftMaster(leftMaster), _rightMaster(rightMaster), _pidgey(pidgey),
	_leftMasterSim(leftMaster.GetSimCollection()), _rightMasterSim(rightMaster.GetSimCollection()), _pidgeySim(pidgey.GetSimCollection()),
	_odometry{pidgey.GetRotation2d()}
{}

/**
 * Returns a 2D representation of the game field for dashboards.
 */
frc::Field2d& DrivebaseSimFX::GetField() {
	return _field;
}

/**
 * Runs the drivebase simulator.
 */
void DrivebaseSimFX::Run() {
	/* Pass the robot battery voltage to the simulated Talon FXs */
	_leftMasterSim.SetBusVoltage(frc::RobotController::GetInputVoltage());
	_rightMasterSim.SetBusVoltage(frc::RobotController::GetInputVoltage());

	/*
	 * CTRE simulation is low-level, so SimCollection inputs
	 * and outputs are not affected by SetInverted(). Only
	 * the regular user-level API calls are affected.
	 *
	 * WPILib expects +V to be forward.
	 * Positive motor output lead voltage is ccw. We observe
	 * on our physical robot that this is reverse for the
	 * right motor, so negate it.
	 *
	 * We are hard-coding the negation of the values instead of
	 * using GetInverted() so we can catch a possible bug in the
	 * robot code where the wrong value is passed to SetInverted().
	 */
	_driveSim.SetInputs(_leftMasterSim.GetMotorOutputLeadVoltage() * 1_V,
						-_rightMasterSim.GetMotorOutputLeadVoltage() * 1_V);

	/*
	 * Advance the model by 20 ms. Note that if you are running this
	 * subsystem in a separate thread or have changed the nominal
	 * timestep of TimedRobot, this value needs to match it.
	 */
	_driveSim.Update(20_ms);

	/*
	 * Update all of our sensors.
	 *
	 * Since WPILib's simulation class is assuming +V is forward,
	 * but -V is forward for the right motor, we need to negate the
	 * position reported by the simulation class. Basically, we
	 * negated the input, so we need to negate the output.
	 */
	_leftMasterSim.SetIntegratedSensorRawPosition(
					DistanceToNativeUnits(
					    _driveSim.GetLeftPosition()
                    ));
	_leftMasterSim.SetIntegratedSensorVelocity(
					VelocityToNativeUnits(
					    _driveSim.GetLeftVelocity()
                    ));
	_rightMasterSim.SetIntegratedSensorRawPosition(
					DistanceToNativeUnits(
					    -_driveSim.GetRightPosition()
                    ));
	_rightMasterSim.SetIntegratedSensorVelocity(
					VelocityToNativeUnits(
					    -_driveSim.GetRightVelocity()
                    ));
	_pidgeySim.SetRawHeading(_driveSim.GetHeading().Degrees().value());

	/*
	 * This will get the simulated sensor readings that we set
	 * in the previous article while in simulation, but will use
	 * real values on the robot itself.
	 */
	_odometry.Update(_pidgey.GetRotation2d(),
					NativeUnitsToDistanceMeters(_leftMaster.GetSelectedSensorPosition()),
					NativeUnitsToDistanceMeters(_rightMaster.GetSelectedSensorPosition()));
	_field.SetRobotPose(_odometry.GetPose());
}

// Helper methods to convert between meters and native units

int DrivebaseSimFX::DistanceToNativeUnits(units::meter_t position){
	double wheelRotations = position/(2 * wpi::numbers::pi * kWheelRadiusInches);
	double motorRotations = wheelRotations * kSensorGearRatio;
	int sensorCounts = (int)(motorRotations * kCountsPerRev);
	return sensorCounts;
}

int DrivebaseSimFX::VelocityToNativeUnits(units::meters_per_second_t velocity){
	auto wheelRotationsPerSecond = velocity/(2 * wpi::numbers::pi * kWheelRadiusInches);
	auto motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
	double motorRotationsPer100ms = motorRotationsPerSecond * 1_s / k100msPerSecond;
	int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
	return sensorCountsPer100ms;
}

units::meter_t DrivebaseSimFX::NativeUnitsToDistanceMeters(double sensorCounts){
	double motorRotations = (double)sensorCounts / kCountsPerRev;
	double wheelRotations = motorRotations / kSensorGearRatio;
	units::meter_t position = wheelRotations * (2 * wpi::numbers::pi * kWheelRadiusInches);
	return position;
}
