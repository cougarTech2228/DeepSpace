package frc.robot.LEDUtilities;

import frc.robot.LEDUtilities.*;
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