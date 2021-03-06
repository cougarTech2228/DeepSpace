package frc.robot.LEDUtilities;

import frc.robot.LEDUtilities.*;
import com.ctre.phoenix.ILoopable;

public class TaskMainLoop implements ILoopable {

	/* ILoopable */
	public void onStart() {
		/* Default to LED strip animation */
		Schedulers.PeriodicTasks.start(Tasks.taskAnimateLEDStrip);
		Schedulers.PeriodicTasks.stop(Tasks.taskDirectControlArm);
		Schedulers.PeriodicTasks.stop(Tasks.taskLIDAR_ControlLEDStrip);
	}

	public void onStop() { }

	public boolean isDone() { return false; }

	public void onLoop() {
        /**
		 * Don't have the ability to check if game-pad is connected...
         * manually enable byt setting gamepadPresent to true;
		 */
        boolean gamepadPresent = false;
		if (gamepadPresent) {
			Schedulers.PeriodicTasks.start(Tasks.taskPWMmotorController);
		} else {
			Schedulers.PeriodicTasks.stop(Tasks.taskPWMmotorController);
		}

		if (Hardware.gamepad.getRawButton(6)) {         // Right-Bumper-Button
			/* Roll through color wheel */
			Schedulers.PeriodicTasks.start(Tasks.taskAnimateLEDStrip);
			Schedulers.PeriodicTasks.stop(Tasks.taskDirectControlArm);
			Schedulers.PeriodicTasks.stop(Tasks.taskLIDAR_ControlLEDStrip);
		} else if (Hardware.gamepad.getRawButton(5)) {  // Left-Bumper-Button
			/* Let user control LED with sticks */
			Schedulers.PeriodicTasks.stop(Tasks.taskAnimateLEDStrip);
			Schedulers.PeriodicTasks.start(Tasks.taskDirectControlArm);
			Schedulers.PeriodicTasks.stop(Tasks.taskLIDAR_ControlLEDStrip);
            /* Measure all PWM Inputs */
			Schedulers.PeriodicTasks.start(Tasks.taskMeasurePulseSensors);
		} else if (Hardware.gamepad.getRawButton(7)) {  // Left-Trigger-Button
			/* LED's controlled with the use of LIDAR sensor */
			Schedulers.PeriodicTasks.stop(Tasks.taskAnimateLEDStrip);
			Schedulers.PeriodicTasks.stop(Tasks.taskDirectControlArm);
			Schedulers.PeriodicTasks.start(Tasks.taskLIDAR_ControlLEDStrip);
            /* Measure all PWM Inputs */
			Schedulers.PeriodicTasks.start(Tasks.taskMeasurePulseSensors);
		}
	}
}