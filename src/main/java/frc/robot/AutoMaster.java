package frc.robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.IterativeRobotBase;

public class AutoMaster {
    public static CommandGroup autoSequence; 
    private DriveBase base;
    public AutoMaster(DriveBase base, Navx navx) {
        this.base = base;
    }
    public void start() {
        Scheduler.getInstance().removeAll();
        autoSequence.addSequential(base.driveToEncoder(100000, 0.3));
        autoSequence.start();
        autoSequence.close();
    }
    public void run() {
        Scheduler.getInstance().run();
    }
}