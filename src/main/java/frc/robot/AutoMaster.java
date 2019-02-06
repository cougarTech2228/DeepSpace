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
    private CommandGroup autoDeployGroup;
    public AutoMaster(DriveBase base, Navx navx, Hatch hatch) {
        visionDataTableInst = NetworkTableInstance.getDefault();
        visionDataTable = visionDataTableInst.getTable(TABLE_KEY);
        targState = visionDataTable.getEntry("targState");
        distTargIn = visionDataTable.getEntry("distTargetIn");
        horzOffToIn = visionDataTable.getEntry("horzOffToIn");
        this.base = base;
        this.hatch = hatch;
        autoDeployGroup = new AutoDeploySequence();
        //autoSequence = new CommandGroup();
    }
    public void start() {
        if(!autoDeployGroup.isCompleted()){
        autoDeployGroup.cancel();
        Scheduler.getInstance().removeAll();
        autoDeployGroup.start();
        //autoSequence.close();
        }
    }
    public void run() {
        Scheduler.getInstance().run();
    }
    public class AutoDeploySequence extends CommandGroup{
        public AutoDeploySequence(){
            this.addSequential(hatch.getHome());
            this.addSequential(hatch.hatchMove(horzOffToIn.getDouble(DEFAULT_VALUE)));
            this.addSequential(base.driveToInch(distTargIn.getDouble(DEFAULT_VALUE), -0.4));
            this.addSequential(hatch.hatchDeploy(2));
        }
        @Override
        protected void initialize() {
        }
        @Override
        public void execute() {
        }

    }
}