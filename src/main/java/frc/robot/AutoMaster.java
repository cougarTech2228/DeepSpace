package frc.robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;

public class AutoMaster {
    DriveBase base;
    Navx navx;
    public AutoMaster(DriveBase base, Navx navx) {
        this.base = base;
        this.navx = navx;
    }
    public void start() {
        Scheduler.getInstance().removeAll();

        CommandGroup Cg = new CommandGroup();
        Cg.addSequential(base.TurnToAngle(navx, 360, 0.5));
        Cg.start();
    }
    public void run() {
        Scheduler.getInstance().run();
    }
}