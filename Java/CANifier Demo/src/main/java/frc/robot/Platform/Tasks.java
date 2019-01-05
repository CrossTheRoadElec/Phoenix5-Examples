/** 
 * Simple Class contatining Tasks used throughout project
 * 
 * Tasks are ILoopable Class Files that can be inserted inside a scheulder
 * for processing. A task could be all the functionality of a subsystem
 * like a robot arm, or could be an individual action to a subsystem, such
 * as up reading a game-pad and applying it to the drive-train. Use 'public
 * static' because these are single objects.
*/
package frc.robot.Platform;

import frc.robot.Tasks.TaskAnimateLEDStrip;
import frc.robot.Tasks.TaskDirectControlLEDStrip;
import frc.robot.Tasks.TaskHSV;
import frc.robot.Tasks.TaskLIDAR_ControlLEDStrip;
import frc.robot.Tasks.TaskMeasurePulseSensors;
import frc.robot.Tasks.TaskPWMmotorController;
import frc.robot.Tasks.TaskMainLoop;
import com.ctre.phoenix.ILoopable;

public class Tasks {
	public static TaskAnimateLEDStrip taskAnimateLEDStrip = new TaskAnimateLEDStrip();
    public static TaskDirectControlLEDStrip taskDirectControlArm = new TaskDirectControlLEDStrip();
    public static TaskHSV taskHSV_ControlLedStrip = new TaskHSV();
    public static TaskLIDAR_ControlLEDStrip taskLIDAR_ControlLEDStrip = new TaskLIDAR_ControlLEDStrip();
    public static TaskMeasurePulseSensors taskMeasurePulseSensors = new TaskMeasurePulseSensors();
	public static TaskPWMmotorController taskPWMmotorController = new TaskPWMmotorController();
	public static TaskMainLoop taskMainLoop = new TaskMainLoop();

	/**
	 * Insert all Tasks below in the Full List so they get auto inserted, 
     * see Robot.java to see how this works.
	 */
	public static ILoopable[] FullList = {  taskAnimateLEDStrip,
                                            taskDirectControlArm, 
                                            taskPWMmotorController,
                                            taskMeasurePulseSensors, 
                                            taskLIDAR_ControlLEDStrip,
                                            taskHSV_ControlLedStrip, 
                                            taskMainLoop,};
}
