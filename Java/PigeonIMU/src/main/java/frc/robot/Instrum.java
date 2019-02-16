/**
 * Instrumentation Class for printing
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Instrum {
	/* Tracking variables for instrumentation */
	static int _printCounter = 0;
	
	public static void Process(PigeonIMU pidgey, BaseMotorController rightSideTalon, boolean printEnable, int signalSelection, int axisSelection) {

		/* track loops since last print */
		if (_printCounter < 0xFFFF) {
			++_printCounter;
		}

		/* if printing is off, leave immedietely */
		if (printEnable == false)
			return;

		if (_printCounter < 50) {
			/* not enough time since last print */
		} else {
			/* rest for next loop */
			_printCounter = 0;

			/* print the selected signals */
			switch (signalSelection) {
				case 0:
					/** Printing YPR Values with Talon */
					double[] ypr = new double[3];
					pidgey.getYawPitchRoll(ypr);

					System.out.println("Printing Yaw/Pitch/Roll");
					System.out.println("Pigeon value: " + ypr[axisSelection]);
					if (rightSideTalon != null)
						System.out.println("Talon Selected Sensor Value: " + rightSideTalon.getSelectedSensorPosition());
					else
						System.out.println("No Talon setup for this platform");
					System.out.println();
					break;
				case 1:
					/** Printing Quaternions */
					double[] quaternions = new double[4];
					pidgey.get6dQuaternion(quaternions);
					System.out.println("Printing Quaternion Values");
					System.out.println("W: " + quaternions[0]);
					System.out.println("X: " + quaternions[1]);
					System.out.println("Y: " + quaternions[2]);
					System.out.println("Z: " + quaternions[3]);
					System.out.println();
					break;
				case 2:
					/** Printing Accumulated Gyro */
					double[] accumGyro = new double[3];
					pidgey.getAccumGyro(accumGyro);
					System.out.println("X: " + accumGyro[0]);
					System.out.println("Y: " + accumGyro[1]);
					System.out.println("Z: " + accumGyro[2]);
					System.out.println();
					break;
				case 3:
					/** Printing Biased Accelerometer Angles */
					short[] biasedAccel = new short[3];
					pidgey.getBiasedAccelerometer(biasedAccel);
					System.out.println("X: " + biasedAccel[0]);
					System.out.println("Y: " + biasedAccel[1]);
					System.out.println("Z: " + biasedAccel[2]);
					System.out.println();
					break;
				case 4:
					/** Printing Raw Gyro */
					double[] rawGyro = new double[3];
					pidgey.getRawGyro(rawGyro);
					System.out.println("X: " + rawGyro[0]);
					System.out.println("Y: " + rawGyro[1]);
					System.out.println("Z: " + rawGyro[2]);
					System.out.println();
					break;
				case 5:
					/** Printing Accelerometer Angles */
					double[] accelAngles = new double[3];
					pidgey.getAccelerometerAngles(accelAngles);
					System.out.println("X: " + accelAngles[0]);
					System.out.println("Y: " + accelAngles[1]);
					System.out.println("Z: " + accelAngles[2]);
					System.out.println();
					break;
				case 6:
					/** Printing Biased Magnetometer Angles */
					short[] biasedMagnet = new short[3];
					pidgey.getBiasedMagnetometer(biasedMagnet);
					System.out.println("X: " + biasedMagnet[0]);
					System.out.println("Y: " + biasedMagnet[1]);
					System.out.println("Z: " + biasedMagnet[2]);
					System.out.println();
					break;
				case 7:
					/** Printing Raw Magnetometer Angles */
					short[] rawMagnet = new short[3];
					pidgey.getRawMagnetometer(rawMagnet);
					System.out.println("X: " + rawMagnet[0]);
					System.out.println("Y: " + rawMagnet[1]);
					System.out.println("Z: " + rawMagnet[2]);
					System.out.println();
					break;
				}
		}
	}
	
	public static void PrintSelection(int pigeonInformationUsed) {
		/** Toggle using ypr or quaternions/accum signals */
		switch (pigeonInformationUsed) {
		   case 0:
			   /** Use YPR */
			   System.out.println("============================");
			   System.out.println("Reading YPR from Pigeon");
			   System.out.println("============================");
			   break;
		   case 1:
			   /** Use Quaternion */
			   System.out.println("============================");
			   System.out.println("Reading Quaternion from Pigeon");
			   System.out.println("============================");
			   break;
		   case 2:
			   /** Use Accum Gyro */
			   System.out.println("============================");
			   System.out.println("Reading Accum Gyro from Pigeon");
			   System.out.println("============================");
			   break;
		   case 3:
			   /** Use Biased Accel */
			   System.out.println("============================");
			   System.out.println("Reading Biased Accel from Pigeon");
			   System.out.println("============================");
			   break;
		   case 4:
			   /** Use Raw Gyro */
			   System.out.println("============================");
			   System.out.println("Reading Raw Gyro from Pigeon");
			   System.out.println("============================");
			   break;
		   case 5:
			   /** Use Accelerometer Angles */
			   System.out.println("============================");
			   System.out.println("Reading Accelerometer from Pigeon");
			   System.out.println("============================");
			   break;
		   case 6:
			   /** Use Biased Magnetometer */
			   System.out.println("============================");
			   System.out.println("Reading Biased Magnetometer from Pigeon");
			   System.out.println("============================");
			   break;
		   case 7:
			   /** Use Raw Magnetometer */
			   System.out.println("============================");
			   System.out.println("Reading Raw Magnetometer from Pigeon");
			   System.out.println("============================");
			   break;
		   }
   }

}
