/**
 * TaskMeasurePulseSensors Class
 * 
 * ILoopable Task for measuring all PWM Channels on CANifier
 * Demonstates the ability to Read PWM inputs from CANifier when connected to sources.
 */
package frc.robot.Tasks;

import frc.robot.Platform.Hardware;
import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.CANifier;

public class TaskMeasurePulseSensors implements ILoopable {
	double[][] _dutyCycleAndPeriods = new double[][]{new double[]{0, 0}, new double[]{0, 0},
                                                     new double[]{0, 0}, new double[]{0, 0}};

	public double getMeasuredPulseWidthsUs(CANifier.PWMChannel pwmCh) {
		return _dutyCycleAndPeriods[pwmCh.value][0];
	}

	/* ILoopable */
    public void onStart() { }
    
    public void onStop() { }
    
    public boolean isDone() { return false; }
    
	public void onLoop() {
		/* Retrieve PWM from the CANifier connected to our PWM source */
		Hardware.canifier.getPWMInput(CANifier.PWMChannel.PWMChannel0,
                _dutyCycleAndPeriods[0]);
		Hardware.canifier.getPWMInput(CANifier.PWMChannel.PWMChannel1,
				_dutyCycleAndPeriods[1]);
		Hardware.canifier.getPWMInput(CANifier.PWMChannel.PWMChannel2,
				_dutyCycleAndPeriods[2]);
		Hardware.canifier.getPWMInput(CANifier.PWMChannel.PWMChannel3,
				_dutyCycleAndPeriods[3]);
	}

	public String toString() {
		return "TaskMeasurePulseSensors:"   + _dutyCycleAndPeriods[0][0] + " :"
                                            + _dutyCycleAndPeriods[1][0] + " :" 
                                            + _dutyCycleAndPeriods[2][0] + " :" 
                                            + _dutyCycleAndPeriods[3][0];
	}
}
