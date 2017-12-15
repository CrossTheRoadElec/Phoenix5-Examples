package org.usfirst.frc.team3539.robot.Tasks;

import com.ctre.phoenix.ILoopable;
import org.usfirst.frc.team3539.robot.Platform.Schedulers;
import org.usfirst.frc.team3539.robot.Platform.Hardware;
import org.usfirst.frc.team3539.robot.Platform.Tasks;

public class TaskMainLoop implements ILoopable{
	/* ILoopable */
    public void OnStart()
    {
        /* Default to LED strip animation */
        Schedulers.PeriodicTasks.Start(Tasks.taskAnimateLEDStrip);
        Schedulers.PeriodicTasks.Stop(Tasks.taskDirectControlArm);
        Schedulers.PeriodicTasks.Stop(Tasks.taskLIDAR_ControlLEDStrip);
    }
    public void OnStop() {
    	
    }
    public boolean IsDone() {
    	return false;
    }
	public void OnLoop()
    {
        boolean gamepadPresent = false;
    	/* Don't have the ability to check if game-pad is connected, manually enable */
        if (gamepadPresent)
        {
            Schedulers.PeriodicTasks.Start(Tasks.taskPWMmotorController);
        }
        else
        {
            Schedulers.PeriodicTasks.Stop(Tasks.taskPWMmotorController);
        }


        if (Hardware.gamepad.getRawButton(6))
        {
            /* Roll through color wheel*/
            Schedulers.PeriodicTasks.Start(Tasks.taskAnimateLEDStrip);
            Schedulers.PeriodicTasks.Stop(Tasks.taskDirectControlArm);
            Schedulers.PeriodicTasks.Stop(Tasks.taskLIDAR_ControlLEDStrip);
        }
        else if (Hardware.gamepad.getRawButton(5))
        {
            /* Let user control LED with sticks */
            Schedulers.PeriodicTasks.Stop(Tasks.taskAnimateLEDStrip);
            Schedulers.PeriodicTasks.Start(Tasks.taskDirectControlArm);
            Schedulers.PeriodicTasks.Stop(Tasks.taskLIDAR_ControlLEDStrip);

            Schedulers.PeriodicTasks.Start(Tasks.taskMeasurePulseSensors);
        }
        else if (Hardware.gamepad.getRawButton(7))
        {
        	/* LED's controlled with the use of LIDAR sensor */
            Schedulers.PeriodicTasks.Stop(Tasks.taskAnimateLEDStrip);
            Schedulers.PeriodicTasks.Stop(Tasks.taskDirectControlArm);
            Schedulers.PeriodicTasks.Start(Tasks.taskLIDAR_ControlLEDStrip);

            Schedulers.PeriodicTasks.Start(Tasks.taskMeasurePulseSensors);
        }
    }
}
