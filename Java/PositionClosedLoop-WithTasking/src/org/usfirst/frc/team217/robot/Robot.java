package org.usfirst.frc.team217.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

/* Import local class files */
import com.ctre.phoenix.ILoopable;
import org.usfirst.frc.team217.robot.Platform.Tasks;
import org.usfirst.frc.team217.robot.Platform.Schedulers;

public class Robot extends IterativeRobot {	
	@Override
	public void robotInit() {
		/* Initalize robot */
	}
	@Override
	public void teleopInit(){
		/* Add each task to the concurrent scheduler */
		for(ILoopable loop : Tasks.FullList){
			Schedulers.PeriodicTasks.add(loop);
		}
		Schedulers.PeriodicTasks.startAll();
	}
	@Override
	public void teleopPeriodic() {
		/* Runs forever */
		
		Schedulers.PeriodicTasks.process();
	}
}
