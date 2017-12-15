package org.usfirst.frc.team3539.robot.Tasks;

import com.ctre.phoenix.ILoopable;
import com.ctre.phoenix.CANifier;
import org.usfirst.frc.team3539.robot.Platform.Hardware;
import edu.wpi.first.wpilibj.can.CANJNI;
import java.nio.ByteBuffer;

public class TaskMeasurePulseSensors implements ILoopable{
    float[][] _dutyCycleAndPeriods = new float[][] { new float[] { 0, 0 }, new float[] { 0, 0 }, new float[] { 0, 0 }, new float[] { 0, 0 } };
    
    public float GetMeasuredPulseWidthsUs(CANifier.PWMChannel pwmCh)
    {
        return _dutyCycleAndPeriods[pwmCh.value][0];
    }
    
    /* ILoopable */
    public void OnStart(){
    }
    public void OnStop() {
    	
    }
    public boolean IsDone() {
    	return false; 
    }
    public void OnLoop()
    {
    	/* Retrieve PWM from the CANifier connected to our PWM source */
        Hardware.canifier.GetPWMInput(CANifier.PWMChannel.PWMChannel0, _dutyCycleAndPeriods[0]);
        Hardware.canifier.GetPWMInput(CANifier.PWMChannel.PWMChannel1, _dutyCycleAndPeriods[1]);
        Hardware.canifier.GetPWMInput(CANifier.PWMChannel.PWMChannel2, _dutyCycleAndPeriods[2]);
        Hardware.canifier.GetPWMInput(CANifier.PWMChannel.PWMChannel3, _dutyCycleAndPeriods[3]);

        /* Send CAN data */
        ByteBuffer data = ByteBuffer.allocateDirect(4);
        data.asIntBuffer().put((int)(_dutyCycleAndPeriods[3][0] * 1000));
        CANJNI.FRCNetCommCANSessionMuxSendMessage(0x1E040000, data, 4);
    }

    public String ToString()
    {
        return "TaskMeasurePulseSensors:" +
            _dutyCycleAndPeriods[0][0] + " :" +
            _dutyCycleAndPeriods[1][0] + " :" +
            _dutyCycleAndPeriods[2][0] + " :" +
            _dutyCycleAndPeriods[3][0];
    }
}
