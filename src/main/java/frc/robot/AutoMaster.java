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
    private Toggler hatchDeployAutoToggler, hatchRetrieveAutoToggler;
    private CommandGroup autoDeployGroup, autoRetrieveGroup;

    public AutoMaster(DriveBase base, Hatch hatch, Vision vision, DriverIF driverIF) {

        this.base = base;
        this.vision = vision;
        this.hatch = hatch;
        this.controller = driverIF;
        this.hatchDeployAutoToggler = new Toggler(2, true);
        this.hatchRetrieveAutoToggler = new Toggler(2, true);
        autoDeployGroup = new AutoDeploy();
        autoRetrieveGroup = new AutoRetrieve();
    }

    public void init() {

        
    }

    public void teleop() {
        hatchDeployAutoToggler.toggle(controller.autoDeploy());
        hatchRetrieveAutoToggler.toggle(controller.autoRetrieve());
        if (controller.autoDeploy()) {
            if (hatchDeployAutoToggler.state == 1 && !autoDeployGroup.isRunning() && !autoRetrieveGroup.isRunning()) {
                System.out.println("Starting auto hatch alignment from button press");
                autoDeployGroup.start();
            } else if (hatchDeployAutoToggler.state == 0 && autoDeployGroup.isRunning()) {
                System.out.println("Canceling auto deploy");
                autoDeployGroup.cancel();
            }
        }
        if (controller.autoRetrieve()) {
            if (hatchRetrieveAutoToggler.state == 1 && !autoRetrieveGroup.isRunning() && !autoDeployGroup.isRunning()) {
                System.out.println("Starting autoRetrieve");
                autoRetrieveGroup.start();
            } else if (hatchRetrieveAutoToggler.state == 0 && autoRetrieveGroup.isRunning()) {
                System.out.println("Canceling autoRetrieve");
                autoRetrieveGroup.cancel();
            }
        }

    }

    public class AutoRetrieve extends CommandGroup {
        public AutoRetrieve() {
            this.addSequential(hatch.hatchMoveCurrent());
            this.addSequential(base.moveToInches(vision.getDistanceFromTarget() - 1, 0.4), 3);
            this.addSequential(base.moveToInches(-3, 0.4));
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
            hatchDeployAutoToggler.state = 0;
            super.end();
        }
    }

    public class AutoDeploy extends CommandGroup {

        public AutoDeploy() {
            this.addSequential(hatch.hatchMoveCurrent());
            this.addSequential(base.moveToInches(vision.getDistanceFromTarget() - 1,
            0.4), 3);
            this.addSequential(hatch.hatchDeploy(0.1));
            this.addSequential(base.moveToInches(-3, 0.4));
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
            hatchDeployAutoToggler.state = 0;
            super.end();
        }
    }
}