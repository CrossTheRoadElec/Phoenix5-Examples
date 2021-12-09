package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DrivebaseSimFX {
	private WPI_TalonFX _leftMaster, _rightMaster;
	private WPI_Pigeon2 _pidgey;

	private TalonFXSimCollection _leftMasterSim, _rightMasterSim;
	private BasePigeonSimCollection _pidgeySim;

	private Field2d _field = new Field2d();
	private DifferentialDriveOdometry _odometry;

	//These numbers are an example AndyMark Drivetrain with some additional weight.  This is a fairly light robot.
	//Note you can utilize results from robot characterization instead of theoretical numbers.
	//https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
	private final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
	private final double kSensorGearRatio = 1; //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
	private final double kGearRatio = 10.71; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
	private final double kWheelRadiusInches = 3;
	private final int k100msPerSecond = 10;

	//Simulation model of the drivetrain
	private DifferentialDrivetrainSim _driveSim = new DifferentialDrivetrainSim(
		DCMotor.getFalcon500(2),  //2 Falcon 500s on each side of the drivetrain.
		kGearRatio,               //Standard AndyMark Gearing reduction.
		2.1,                      //MOI of 2.1 kg m^2 (from CAD model).
		26.5,                     //Mass of the robot is 26.5 kg.
		Units.inchesToMeters(kWheelRadiusInches),  //Robot uses 3" radius (6" diameter) wheels.
		0.546,                    //Distance between wheels is _ meters.
		
		// The standard deviations for measurement noise:
		// x and y:          0.001 m
		// heading:          0.001 rad
		// l and r velocity: 0.1   m/s
		// l and r position: 0.005 m
		null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
	);

	/**
	 * Creates a new drivebase simualtor using Falcon 500 motors.
	 * 
	 * @param leftMaster the left master Falcon
	 * @param rightMaster the right master Falcon
	 * @param pidgey the Pigeon IMU
	 */
	public DrivebaseSimFX(WPI_TalonFX leftMaster, WPI_TalonFX rightMaster, WPI_Pigeon2 pidgey) {
		_leftMaster = leftMaster;
		_rightMaster = rightMaster;
		_pidgey = pidgey;

		_leftMasterSim = leftMaster.getSimCollection();
		_rightMasterSim = rightMaster.getSimCollection();
		_pidgeySim = pidgey.getSimCollection();

		// Creating odometry object. Here,
		// our starting pose is 5 meters along the long end of the field and in the
		// center of the field along the short end, facing forward.
		_odometry = new DifferentialDriveOdometry(_pidgey.getRotation2d());
	}

	/**
	 * Returns a 2D representation of the game field for dashboards.
	 */
	public Field2d getField() {
		return _field;
	}

	/**
	 * Runs the drivebase simulator.
	 */
	public void run() {
		// Set the inputs to the system. Note that we need to use
		// the output voltage, NOT the percent output.
		_driveSim.setInputs(_leftMasterSim.getMotorOutputLeadVoltage(),
							-_rightMasterSim.getMotorOutputLeadVoltage()); //Right side is inverted, so forward is negative voltage
	
		// Advance the model by 20 ms. Note that if you are running this
		// subsystem in a separate thread or have changed the nominal timestep
		// of TimedRobot, this value needs to match it.
		_driveSim.update(0.02);
	
		// Update all of our sensors.
		_leftMasterSim.setIntegratedSensorRawPosition(
						distanceToNativeUnits(
						_driveSim.getLeftPositionMeters()));
		_leftMasterSim.setIntegratedSensorVelocity(
						velocityToNativeUnits(
						_driveSim.getLeftVelocityMetersPerSecond()));
		_rightMasterSim.setIntegratedSensorRawPosition(
						distanceToNativeUnits(
						-_driveSim.getRightPositionMeters()));
		_rightMasterSim.setIntegratedSensorVelocity(
						velocityToNativeUnits(
						-_driveSim.getRightVelocityMetersPerSecond()));
		_pidgeySim.setRawHeading(_driveSim.getHeading().getDegrees());
	
		//Update other inputs to Talons
		_leftMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
		_rightMasterSim.setBusVoltage(RobotController.getBatteryVoltage());
		
		// This will get the simulated sensor readings that we set
		// in the previous article while in simulation, but will use
		// real values on the robot itself.
		_odometry.update(_pidgey.getRotation2d(),
							nativeUnitsToDistanceMeters(_leftMaster.getSelectedSensorPosition()),
							nativeUnitsToDistanceMeters(_rightMaster.getSelectedSensorPosition()));
		_field.setRobotPose(_odometry.getPoseMeters());
	}

	// Helper methods to convert between meters and native units
  
	private int distanceToNativeUnits(double positionMeters){
		double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
		double motorRotations = wheelRotations * kSensorGearRatio;
		int sensorCounts = (int)(motorRotations * kCountsPerRev);
		return sensorCounts;
	}
  
	private int velocityToNativeUnits(double velocityMetersPerSecond){
		double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
		double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
		double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
		int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
		return sensorCountsPer100ms;
	}
  
	private double nativeUnitsToDistanceMeters(double sensorCounts){
		double motorRotations = (double)sensorCounts / kCountsPerRev;
		double wheelRotations = motorRotations / kSensorGearRatio;
		double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
		return positionMeters;
	}
}
