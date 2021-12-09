#include "DrivebaseSimSRX.h"
#include <wpi/numbers>

/**
 * Creates a new drivebase simualtor using Talon SRX motor controllers.
 * 
 * @param leftMaster the left master Talon SRX
 * @param rightMaster the right master Talon SRX
 * @param pidgey the Pigeon IMU
 */
DrivebaseSimSRX::DrivebaseSimSRX(WPI_TalonSRX& leftMaster, WPI_TalonSRX& rightMaster, WPI_PigeonIMU& pidgey)
	: _leftMaster(leftMaster), _rightMaster(rightMaster), _pidgey(pidgey),
	_leftMasterSim(leftMaster.GetSimCollection()), _rightMasterSim(rightMaster.GetSimCollection()), _pidgeySim(pidgey.GetSimCollection()),
	_odometry{pidgey.GetRotation2d()}
{}

/**
 * Returns a 2D representation of the game field for dashboards.
 */
frc::Field2d& DrivebaseSimSRX::GetField() {
	return _field;
}

/**
 * Runs the drivebase simulator.
 */
void DrivebaseSimSRX::Run() {
	// Set the inputs to the system. Note that we need to use
	// the output voltage, NOT the percent output.
	_driveSim.SetInputs(units::volt_t{_leftMasterSim.GetMotorOutputLeadVoltage()},
						units::volt_t{-_rightMasterSim.GetMotorOutputLeadVoltage()}); //Right side is inverted, so forward is negative voltage

	// Advance the model by 20 ms. Note that if you are running this
	// subsystem in a separate thread or have changed the nominal timestep
	// of TimedRobot, this value needs to match it.
	_driveSim.Update(0.02_s);

	// Update all of our sensors.
	_leftMasterSim.SetQuadratureRawPosition(
					DistanceToNativeUnits(
					_driveSim.GetLeftPosition().to<double>()));
	_leftMasterSim.SetQuadratureVelocity(
					VelocityToNativeUnits(
					_driveSim.GetLeftVelocity().to<double>()));
	// Sensor phase is true, invert encoder directions
	_rightMasterSim.SetQuadratureRawPosition(
					DistanceToNativeUnits(
					-_driveSim.GetRightPosition().to<double>()));
	_rightMasterSim.SetQuadratureVelocity(
					VelocityToNativeUnits(
					-_driveSim.GetRightVelocity().to<double>()));
	_pidgeySim.SetRawHeading(_driveSim.GetHeading().Degrees().to<double>());

	//Update other inputs to Talons
	_leftMasterSim.SetBusVoltage(frc::RobotController::GetInputVoltage());
	_rightMasterSim.SetBusVoltage(frc::RobotController::GetInputVoltage());

	// This will get the simulated sensor readings that we set
	// in the previous article while in simulation, but will use
	// real values on the robot itself.
	_odometry.Update(_pidgey.GetRotation2d(),
					units::meter_t{NativeUnitsToDistanceMeters(_leftMaster.GetSelectedSensorPosition())},
					units::meter_t{NativeUnitsToDistanceMeters(_rightMaster.GetSelectedSensorPosition())});
	_field.SetRobotPose(_odometry.GetPose());
}

// Helper methods to convert between meters and native units

int DrivebaseSimSRX::DistanceToNativeUnits(double positionMeters){
	double wheelRotations = positionMeters/(2 * wpi::numbers::pi * kWheelRadiusInches.convert<units::meter>().to<double>());
	double motorRotations = wheelRotations * kSensorGearRatio;
	int sensorCounts = (int)(motorRotations * kCountsPerRev);
	return sensorCounts;
}

int DrivebaseSimSRX::VelocityToNativeUnits(double velocityMetersPerSecond){
	double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * wpi::numbers::pi * kWheelRadiusInches.convert<units::meter>().to<double>());
	double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
	double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
	int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
	return sensorCountsPer100ms;
}

double DrivebaseSimSRX::NativeUnitsToDistanceMeters(double sensorCounts){
	double motorRotations = (double)sensorCounts / kCountsPerRev;
	double wheelRotations = motorRotations / kSensorGearRatio;
	double positionMeters = wheelRotations * (2 * wpi::numbers::pi * kWheelRadiusInches.convert<units::meter>().to<double>());
	return positionMeters;
}
