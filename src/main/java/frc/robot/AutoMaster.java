package frc.robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.IterativeRobotBase;

public class AutoMaster {
    DriveBase base;
    public AutoMaster(DriveBase base, Navx navx) {
        this.base = base;
    }
    public void start() {
        Scheduler.getInstance().removeAll();

        CommandGroup Cg = new CommandGroup();
        CommandGroup cg2 = new CommandGroup();
        //cg2
        cg2.addSequential(base.TurnToAngle(360, 0.5));
        //Cg.addSequential();
        Cg.addSequential(base.driveToEncoder(100000, 0.3));
        Cg.start();
        Cg.close();
        System.out.println("initialized ar0fsaezuihel");
    }
    public void run() {
        Scheduler.getInstance().run();
    }
}