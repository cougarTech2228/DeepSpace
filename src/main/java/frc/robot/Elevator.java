
package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private boolean doAutoClimb = false;
    private boolean deploy = false;
    private boolean wasAutoButtonPressed = false;
    private boolean doManualClimb = false;
    private boolean wasManualButtonPressed = false;
    private boolean isStallCurrentActive = false;

    private DriverIF controls;
    private DriveBase base;

    private SerialDataHandler arduino = new SerialDataHandler();

    // Lift Drive encoder math
    private double circumference = 4 * Math.PI;
    private int gearRatio = 63;
    private double liftDriveMovePerRev = 0.1994;
    private double liftDriveEncodersPerRev = circumference * gearRatio / liftDriveMovePerRev;

    // Drive Motor encoder math
    private double driveCircumference = 6 * Math.PI;
    private double driveGearRatio = (12.0 * 34.0) / (50.0 * 50.0);
    private double driveMovePerRev = 0.1994;
    // ^^^ Copied from the liftDrive cuz I'm lazy ^^^
    private double driveEncodersPerRev = driveCircumference * driveGearRatio / driveMovePerRev;

    private double leftDistance = 0;
    private double rightDistance = 0;
    private double mmToIn = 0.0393701;
    private double maxDistanceToPlatform = 0;

    // The front of the bot is where the hatch mechanism is
    private Motor frontLift;
    private Motor backLift;
    private Motor liftDrive;
    private Motor elevatorDeployMotor;

    // Sensors
    private DigitalInput frontLiftRaised = new DigitalInput(RobotMap.DIGITAL_INPUT_2);
    private DigitalInput frontLiftLowered = new DigitalInput(RobotMap.DIGITAL_INPUT_3);
    private DigitalInput backLiftRaised = new DigitalInput(RobotMap.DIGITAL_INPUT_4);
    private DigitalInput backLiftLowered = new DigitalInput(RobotMap.DIGITAL_INPUT_5);
    private DigitalInput elevatorDeploy = new DigitalInput(RobotMap.DIGITAL_INPUT_6);

    private double frontLiftSpeedUp = 0.5;
    private double frontLiftSpeedDown = -0.4;
    private double backLiftSpeedUp = 0.5;
    private double backLiftSpeedDown = -0.4;
    private double liftDriveSpeed = 0.2;

    private double closedLoopSpeed = 0;

    private double startTime;

    private int encoderCount = 0;

    private autoClimb autoClimbState = autoClimb.PullRobotUp;

    private enum autoClimb {
        PullRobotUp, MoveForward, LiftDriveMotorUp, MoveFullyForward
    }

    private manualClimb manualClimbState = manualClimb.PullRobotUp;

    private enum manualClimb {
        PullRobotUp, MoveForward, LiftDriveMotorUp, MoveFullyForward
    }

    private climb climbState = climb.PullRobotUp;

    private enum climb {
        PullRobotUp, MoveForward, LiftDriveMotorUp, MoveFullyForward
    }

    public Elevator(DriveBase driver, DriverIF controls) {
        base = driver;
        frontLift = new Motor(RobotMap.ACTION_MOTOR_1);
        backLift = new Motor(RobotMap.ACTION_MOTOR_2);
        liftDrive = new Motor(RobotMap.ACTION_MOTOR_3);
        elevatorDeployMotor = new Motor(RobotMap.ACTION_MOTOR_4);
        this.controls = controls;
    }

    public void teleopRaise() {
        SmartDashboard.putBoolean("Front Raised", frontLiftRaised.get());
        SmartDashboard.putBoolean("Front Lowered", frontLiftLowered.get());
        SmartDashboard.putBoolean("Back Raised", backLiftRaised.get());
        SmartDashboard.putBoolean("Back Lowered", backLiftLowered.get());
        SmartDashboard.putBoolean("Elevator Deployed", elevatorDeploy.get());

        raiseElevator();

        if (controls.manualClimb() && wasManualButtonPressed == false) {
            if (doManualClimb == true) {
                doManualClimb = false;
                liftDrive.setEncoderToZero();
            } else {
                doManualClimb = true;
                doAutoClimb = false;
                startTime = Timer.getFPGATimestamp();
                System.out.println("Climbing Up----------------------------------------");
                manualClimbState = manualClimb.PullRobotUp;
            }
            wasManualButtonPressed = true;
        } else if (!controls.manualClimb()) {
            wasManualButtonPressed = false;
        }

        // -----------------------------AUTOCLIMB
        // CODE---------------------------------------------------------
        if (controls.autoClimb() && wasAutoButtonPressed == false) {
            if (doAutoClimb == true) {
                doAutoClimb = false;
                liftDrive.setEncoderToZero();
            } else {
                doAutoClimb = true;
                doManualClimb = false;
                startTime = Timer.getFPGATimestamp();
                System.out.println("Auto Climbing Up----------------------------------------");
                autoClimbState = autoClimb.PullRobotUp;
            }
            wasAutoButtonPressed = true;
        } else if (!controls.autoClimb()) {
            wasAutoButtonPressed = false;
        }

        // Climbing State Machine

        if (doAutoClimb || doManualClimb) {
            switch (climbState) {

            case PullRobotUp:
                if (doManualClimb) {
                    if (frontLiftLowered.get()) {
                        frontLift.set(frontLiftSpeedDown);
                        System.out.println("Lowering Front");
                    } else {
                        frontLift.set(0);
                    }
                    if (Timer.getFPGATimestamp() - startTime == 5) {
                        backLift.set(backLiftSpeedDown);
                    } else {
                        backLift.set(0);
                    }

                    if (!frontLiftLowered.get() && (Timer.getFPGATimestamp() - startTime == 5)) {
                        climbState = climb.MoveForward;
                        System.out.println("Moving Forward");
                    }
                } else {
                    if (frontLiftLowered.get()) {
                        frontLift.set(frontLiftSpeedDown);
                    } else {
                        frontLift.set(0);
                    }

                    if (backLiftLowered.get()) {
                        backLift.set(-backLiftSpeedDown);
                    } else {
                        backLift.set(0);
                    }

                    if (!frontLiftLowered.get() && !backLiftLowered.get()) {
                        System.out.println("Auto Moving Forward-----------------------------");
                        climbState = climb.MoveForward;
                    }
                }
                break;

            case MoveForward:
                if (doManualClimb) {
                    liftDrive.set(-controls.throttle());
                    base.TeleopMove();
                    if (controls.retractLiftDrive()) {
                        System.out.println("Lifting Drive Motor Up-------------------------------------");
                        climbState = climb.LiftDriveMotorUp;
                    }
                } else {
                    // if (Math.abs(liftDrive.getSensorPosition()) <= liftDriveEncodersPerRev * 20)
                    // {
                    if (elevatorDeploy.get()) {
                        base.elevatorClimb(liftDriveSpeed, liftDriveEncodersPerRev * 20);
                        liftDrive.set(liftDriveSpeed);
                        // System.out.println(liftDrive.getSensorPosition());
                    } else {
                        liftDrive.set(0);
                        base.stopMoving();
                        System.out.println("Auto Lifting The Drive Motor Up");
                        climbState = climb.LiftDriveMotorUp;
                        // }
                    }
                }
                break;

            case LiftDriveMotorUp:

                if (frontLiftRaised.get()) {
                    frontLift.set(frontLiftSpeedUp);
                } else {
                    System.out.println("Moving Forward Again");
                    frontLift.set(0);
                    climbState = climb.MoveFullyForward;
                }

                break;

            case MoveFullyForward:
                if (doManualClimb) {
                    base.TeleopMove();
                    System.out.println("Moving");
                } else {
                    // if (Math.abs(base.platformEncoderLeft()) <= driveEncodersPerRev) {
                    if (elevatorDeploy.get()) {
                        base.elevatorClimb(.3, driveEncodersPerRev);
                        System.out.println("Moving");
                    }

                    else {
                        base.stopMoving();
                        System.out.println("Finished");
                        doAutoClimb = false;
                    }
                    // }

                }
                break;
            }
        }

    }

    public void raiseElevator() {
        SmartDashboard.putNumber("Elevator Deploy Current", elevatorDeployMotor.getMotorCurrent());
        SmartDashboard.putBoolean("Stall Current Activity", isStallCurrentActive);
        // if (controls.deployElevator()) {
        // deploy = true;
        // }

        if (controls.deployElevator()) {

            if (elevatorDeployMotor.getMotorCurrent() >= 3.5) {
                isStallCurrentActive = true;

            }

            if (isStallCurrentActive) {
                elevatorDeployMotor.set(0.15);
                // deploy = false;
            }

            else {
                // if (!elevatorDeploy.get()) {
                elevatorDeployMotor.set(0.2);
            }

            if (backLiftRaised.get()) {
                backLift.set(0.2);
            } else {
                backLift.set(0);
            }
        }

    }

    public class ElevatorCommand extends Command {
        private boolean finished;

        protected void initialize() {
            this.finished = false;
        }

        protected void execute() {
            if (!backLiftRaised.get()) {
                backLift.set(backLiftSpeedUp);
            }

            if (backLiftRaised.get()) {
                backLift.set(0);
                finished = true;
            }
        }

        protected boolean isFinished() {
            return finished;
        }

        public void end() {

        }
    }

    public void testLiftDriveEncoder() {
        System.out.println(liftDrive.getSensorPosition());
    }

}
