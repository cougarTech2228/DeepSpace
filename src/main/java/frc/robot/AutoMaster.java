package frc.robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.IterativeRobotBase;

public class AutoMaster {
    // public static CommandGroup autoSequence;
    private DriveBase base;
    private Vision vision;
    private Hatch hatch;
    private DriverIF controller;
    private Toggler hatchDeployAutoToggler, hatchRetrieveAutoToggler, homeAutoToggler;
    private CommandGroup autoDeployGroup, autoRetrieveGroup;
    private Command autoHome;

    public AutoMaster(DriveBase base, Hatch hatch, Vision vision, DriverIF driverIF) {

        this.base = base;
        this.vision = vision;
        this.hatch = hatch;
        this.controller = driverIF;
        this.hatchDeployAutoToggler = new Toggler(2, true);
        this.hatchRetrieveAutoToggler = new Toggler(2, true);
        this.homeAutoToggler = new Toggler(2, true);
        autoDeployGroup = new AutoDeploy();
        autoRetrieveGroup = new AutoRetrieve();
        autoHome = hatch.getHome();
    }

    public void init() {

    }

    public void teleop() {
        hatchDeployAutoToggler.toggle(controller.autoDeploy());
        hatchRetrieveAutoToggler.toggle(controller.autoRetrieve());
        homeAutoToggler.toggle(controller.btHome());
        if (controller.autoDeploy()) {
            if (hatchDeployAutoToggler.state == 1 && !autoDeployGroup.isRunning() && !autoRetrieveGroup.isRunning()) {
                autoDeployGroup = new AutoDeploy();
                System.out.println("Starting auto hatch alignment from button press");
                autoDeployGroup.start();
            } else if (hatchDeployAutoToggler.state == 0 && autoDeployGroup.isRunning()) {
                System.out.println("Cancelling auto deploy");
                autoDeployGroup.cancel();
            }
        }
        if (controller.autoRetrieve()) {
            if (hatchRetrieveAutoToggler.state == 1 && !autoRetrieveGroup.isRunning() && !autoDeployGroup.isRunning()) {
                autoRetrieveGroup = new AutoRetrieve();
                System.out.println("Starting autoRetrieve");
                autoRetrieveGroup.start();
            } else if (hatchRetrieveAutoToggler.state == 0 && autoRetrieveGroup.isRunning()) {
                System.out.println("Cancelling autoRetrieve");
                autoRetrieveGroup.cancel();
            }
        }
        if (controller.btHome()) {
            if (homeAutoToggler.state == 1 && !autoRetrieveGroup.isRunning() && !autoDeployGroup.isRunning()
                    && !autoHome.isRunning()) {
                autoHome = hatch.getHome();
                System.out.println("Starting autoHome");
                autoHome.start();
            } else if (homeAutoToggler.state == 0) {
                System.out.println("Cancelling autoHome");
                autoHome.cancel();
            }
        }

    }

    public class AutoRetrieve extends CommandGroup {
        public AutoRetrieve() {
            if (vision.getCameraState() == 2) {
                if (vision.getDistanceFromTarget() > 18 && vision.getDistanceFromTarget() < 48) {
                    System.out.println(vision.getDistanceFromTarget());
                    double offset = hatch.getOffset();
                    if (!(offset < 6 && offset > 0)) {
                        System.out.println("Offset is too great: " + offset);
                        this.cancel();
                    } else {
                        this.addSequential(hatch.hatchMoveCurrent());
                        this.addSequential(base.moveToInches(vision.getDistanceFromTarget(), 0.3), 3);
                        this.addSequential(base.moveToInches(-3, 0.4));
                    }
                } else {
                    System.out.println("Not in specified distance");
                    this.cancel();
                }
            } else {
                System.out.println("Not locked");
                this.cancel();
            }
        }

        @Override
        protected void initialize() {
            if (vision.getCameraState() == 2) {
                if (vision.getDistanceFromTarget() > 18 && vision.getDistanceFromTarget() < 48) {
                    System.out.println(vision.getDistanceFromTarget());
                    double offset = hatch.getOffset();
                    if (!(offset < 6 && offset > 0)) {
                        System.out.println("Offset is too great: " + offset);
                        this.cancel();
                    }

                } else {
                    System.out.println("Not in specified distance");
                    this.cancel();
                }
            } else {
                System.out.println("Not locked");
                this.cancel();
            }

        }

        @Override
        protected void interrupted() {
            Scheduler.getInstance().removeAll();
        }

        @Override
        public void end() {
            hatchRetrieveAutoToggler.state = 0;
            super.end();
        }
    }

    public class AutoDeploy extends CommandGroup {

        public AutoDeploy() {
            if (vision.getCameraState() == 2) {
                if (vision.getDistanceFromTarget() > 18 && vision.getDistanceFromTarget() < 48) {
                    System.out.println(vision.getDistanceFromTarget());
                    double offset = hatch.getOffset();
                    if (!(offset < 6 && offset > 0)) {
                        System.out.println("Offset is too great: " + offset);
                        this.cancel();
                    } else {
                        this.addSequential(hatch.hatchMoveCurrent());
                        this.addSequential(base.moveToInches(vision.getDistanceFromTarget(), 0.3), 3);
                        this.addSequential(hatch.hatchDeploy(.5));
                        this.addSequential(base.moveToInches(-3, 0.4));
                    }
                } else {
                    System.out.println("Not in specified distance");
                    this.cancel();
                }
            } else {
                System.out.println("Not locked");
                this.cancel();
            }
        }

        @Override
        protected void initialize() {
            if (vision.getCameraState() == 2) {
                if (vision.getDistanceFromTarget() > 18 && vision.getDistanceFromTarget() < 48) {
                    System.out.println(vision.getDistanceFromTarget());
                    double offset = hatch.getOffset();
                    if (!(offset < 6 && offset > 0)) {
                        System.out.println("Offset is too great: " + offset);
                        this.cancel();
                    }

                } else {
                    System.out.println("Not in specified distance");
                    this.cancel();
                }
            } else {
                System.out.println("Not locked");
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