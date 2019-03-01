package frc.robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.IterativeRobotBase;

public class AutoMaster {
    // public static CommandGroup autoSequence;
    private DriveBase base;
    private Vision vision;
    private Hatch hatch;
    private DriverIF controller;
    private Toggler hatchAutoToggler;
    private CommandGroup autoDeployGroup;

    public AutoMaster(DriveBase base, Hatch hatch, Vision vision, DriverIF driverIF) {

        this.base = base;
        this.vision = vision;
        this.hatch = hatch;
        this.controller = driverIF;
        this.hatchAutoToggler = new Toggler(2, true);
        // autoDeployGroup = new AutoDeploy();
        // autoDeployGroup.addSequential(base.moveToDistancePigeon());//hatch.getAutoDeploy();
        // autoSequence = new CommandGroup();
    }

    public void init() {

        // if (autoDeployGroup.isRunning()) {
        // System.out.println("Already in use");
        // } else {
        // System.out.println("Starting auto");
        // autoDeployGroup.start();
        // }
    }

    public void teleop() {

        // if state has changed, aka button has just been pressed.
        // this if will be run only once per press
        if (hatchAutoToggler.detectChange(controller.autoAlign())) {
            if (hatchAutoToggler.state == 1) {
                autoDeployGroup = new AutoDeploy();
                autoDeployGroup.start();
            } else {
                autoDeployGroup.cancel();
            }
        }

    }

    public class AutoDeploy extends CommandGroup {

        public AutoDeploy() {
            this.addSequential(hatch.hatchMoveCurrent());
            // commented these out so drivers could have aut alignment, they'll have to
            // deploy themselves.
            // this.addSequential(base.moveToInches(vision.getDistanceFromTarget() - 1,
            // 0.4), 3);
            // this.addSequential(hatch.hatchDeploy(0.1));
            // this.addSequential(base.moveToInches(-3, 0.4));
        }

        @Override
        protected void initialize() {
            if (vision.getDistanceFromTarget() > 18 && vision.getDistanceFromTarget() < 48) {
                System.out.println(vision.getDistanceFromTarget());
                double offset = hatch.getOffset();
                if (!(offset < 6 && offset > 0)) {
                    System.out.println("Offset is too great: " + offset);
                    this.cancel();
                }
                if (vision.getCameraState() == 2) {
                } else {
                    System.out.println("Not locked");
                    this.cancel();
                }
            } else {
                System.out.println("Not in specified distance");
                this.cancel();
            }

        }

        @Override
        protected void interrupted() {
            Scheduler.getInstance().removeAll();
        }

        @Override
        public void end() {
            hatchAutoToggler.state = 0;
            super.end();
        }
    }
}