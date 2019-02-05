package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class AutoMaster {
    //public static CommandGroup autoSequence; 
    private DriveBase base;
    private Hatch hatch;
    private final String TABLE_KEY = "datatable";
    private NetworkTableInstance visionDataTableInst;
    private NetworkTable visionDataTable;
    private final int DEFAULT_VALUE = 69420666;
    // state, 0 for searching, 1 for acquiring, or 2 for locked
    private NetworkTableEntry targState;
    // distance to target in inches,
    private NetworkTableEntry distTargIn;
    // horizontal distance from center of target, positive being to the right
    private NetworkTableEntry horzOffToIn;
    public AutoMaster(DriveBase base, Navx navx, Hatch hatch) {
        visionDataTableInst = NetworkTableInstance.getDefault();
        visionDataTable = visionDataTableInst.getTable(TABLE_KEY);
        targState = visionDataTable.getEntry("targState");
        distTargIn = visionDataTable.getEntry("distTargetIn");
        horzOffToIn = visionDataTable.getEntry("horzOffToIn");
        this.base = base;
        this.hatch = hatch;
        //autoSequence = new CommandGroup();
    }
    public void start() {
        //Scheduler.getInstance().removeAll();

        //autoSequence.addSequential(base.driveToEncoder(100000, 0.3));
        /*
        autoSequence.start();
        Scheduler.getInstance().removeAll();
        if (!autoSequence.isCompleted()) autoSequence.cancel();
        //autoDeployGroup.start();
        if (distTargIn.getDouble(DEFAULT_VALUE) < 24) {
            if (targState.getDouble(DEFAULT_VALUE) == 2.0) {
                autoSequence.start();
            } else {
                System.out.println("Not locked on: " + targState.getDouble(DEFAULT_VALUE));
                autoSequence.addSequential(hatch.hatchMove(horzOffToIn.getDouble(0)));
                autoSequence.addSequential(base.driveToEncoder(distTargIn.getDouble(0), 0.4));
                autoSequence.addSequential(hatch.hatchDeploy(2));
            }
        } else {
            System.out.println("Too far: " + distTargIn.getDouble(DEFAULT_VALUE));
        }*/
        //autoSequence.addSequential(base.driveToInch(12, 0.5));
        CommandGroup autoSequence = new CommandGroup();
        System.out.println("initializing");
        autoSequence.addSequential(base.driveToInch(12, 0.5));
        autoSequence.start();
        //autoSequence.close();
    }
    public void run() {
        Scheduler.getInstance().run();
    }
}