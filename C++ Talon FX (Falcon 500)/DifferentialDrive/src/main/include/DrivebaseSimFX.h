#include <frc/RobotController.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <units/units.h>
#include "ctre/Phoenix.h"

class DrivebaseSimFX {
public:
	/**
	 * Creates a new drivebase simualtor using Falcon 500 motors.
	 * 
	 * @param leftMaster the left master Falcon
	 * @param rightMaster the right master Falcon
	 * @param pidgey the Pigeon IMU
	 */
	DrivebaseSimFX(WPI_TalonFX& leftMaster, WPI_TalonFX& rightMaster, WPI_PigeonIMU& pidgey);

	/**
	 * Returns a 2D representation of the game field for dashboards.
	 */
	frc::Field2d& GetField();

	/**
	 * Runs the drivebase simulator.
	 */
	void Run();

private:
	WPI_TalonFX& _leftMaster;
	WPI_TalonFX& _rightMaster;
	WPI_PigeonIMU& _pidgey;

	TalonFXSimCollection& _leftMasterSim;
	TalonFXSimCollection& _rightMasterSim;
	BasePigeonSimCollection& _pidgeySim;

	//These numbers are an example AndyMark Drivetrain with some additional weight.  This is a fairly light robot.
	//Note you can utilize results from robot characterization instead of theoretical numbers.
	//https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
	const int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
	const double kSensorGearRatio = 1; //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
	const double kGearRatio = 10.71; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
	const units::inch_t kWheelRadiusInches = 3_in;
	const int k100msPerSecond = 10;

	//Simulation model of the drivetrain
	frc::sim::DifferentialDrivetrainSim _driveSim{
		frc::DCMotor::Falcon500(2),  //2 Falcon 500s on each side of the drivetrain.
		kGearRatio,                  //Standard AndyMark Gearing reduction.
		2.1_kg_sq_m,                 //MOI of 2.1 kg m^2 (from CAD model).
		26.5_kg,                     //Mass of the robot is 26.5 kg.
		kWheelRadiusInches,          //Robot uses 3" radius (6" diameter) wheels.
		0.546_m,                     //Distance between wheels is _ meters.
		
		// The standard deviations for measurement noise:
		// x and y:          0.001 m
		// heading:          0.001 rad
		// l and r velocity: 0.1   m/s
		// l and r position: 0.005 m
		//VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
	};

	frc::Field2d _field;
	// Creating my odometry object. Here,
	// our starting pose is 5 meters along the long end of the field and in the
	// center of the field along the short end, facing forward.
	frc::DifferentialDriveOdometry _odometry;

	// Helper methods to convert between meters and native units
	int DistanceToNativeUnits(units::meter_t position);
	int VelocityToNativeUnits(units::meters_per_second_t velocity);
	units::meter_t NativeUnitsToDistanceMeters(double sensorCounts);
};
