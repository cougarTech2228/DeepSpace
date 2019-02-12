
package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DriveBase.DriveToInch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private boolean doAutoClimb = false;
    private boolean deploy = false;
    private boolean wasAutoButtonPressed = false;
    private boolean doManualClimb = false;
    private boolean wasManualButtonPressed = false;

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

    // Sensors
    private DigitalInput frontLiftRaised = new DigitalInput(RobotMap.DIGITAL_INPUT_2);
    private DigitalInput frontLiftLowered = new DigitalInput(RobotMap.DIGITAL_INPUT_3);
    private DigitalInput backLiftRaised = new DigitalInput(RobotMap.DIGITAL_INPUT_4);
    private DigitalInput backLiftLowered = new DigitalInput(RobotMap.DIGITAL_INPUT_5);
    private DigitalInput driveSwitch = new DigitalInput(RobotMap.DIGITAL_INPUT_6);

    private double frontLiftSpeedUp = 0.5;
    private double frontLiftSpeedDown = -0.4;
    private double backLiftSpeedUp = 0.5;
    private double backLiftSpeedDown = -0.4;
    private double liftDriveSpeed = 0.2;

    private int encoderCount = 0;

    private autoClimb autoClimbState = autoClimb.PullRobotUp;

    private enum autoClimb {
        PullRobotUp, MoveForward, LiftDriveMotorUp, MoveFullyForward
    }

    private manualClimb manualClimbState = manualClimb.PullRobotUp;

    private enum manualClimb {
        PullRobotUp, MoveForward, LiftDriveMotorUp, MoveFullyForward
    }

    public Elevator(DriveBase driver, DriverIF controls) {
        base = driver;
        frontLift = new Motor(RobotMap.ACTION_MOTOR_1);
        backLift = new Motor(RobotMap.ACTION_MOTOR_2);
        liftDrive = new Motor(RobotMap.ACTION_MOTOR_3);
        this.controls = controls;
    }

    public void teleopRaise() {
        // System.out.println(controls.manualClimb() +
        // "-----------------------------------------");
        // System.out.println(wasManualButtonPressed +
        // "---------------------------------------");
        if (controls.manualClimb() && wasManualButtonPressed == false) {
            if (doManualClimb == true) {
                doManualClimb = false;
                liftDrive.setEncoderToZero();
            } else {
                doManualClimb = true;
                System.out.println("Climbing Up----------------------------------------");
                manualClimbState = manualClimb.PullRobotUp;
            }
            wasManualButtonPressed = true;
        } else if (!controls.manualClimb()) {
            wasManualButtonPressed = false;
        }

        if (doManualClimb) {
            switch (manualClimbState) {

            case PullRobotUp:
                if (frontLiftLowered.get()) {
                    frontLift.set(frontLiftSpeedDown);
                } else {
                    frontLift.set(0);
                }
                if (backLiftLowered.get()) {
                    backLift.set(backLiftSpeedDown);
                } else {
                    backLift.set(0);
                }

                if (!backLiftLowered.get() && !frontLiftLowered.get()) {
                    System.out.println("Moving Forward-----------------------------------");
                    manualClimbState = manualClimb.MoveForward;
                }

                break;

            case MoveForward:
                liftDrive.set(controls.throttle());
                base.TeleopMove();
                if (controls.retractLiftDrive()) {
                    System.out.println("Lifting Drive Motor Up-------------------------------------");
                    manualClimbState = manualClimb.LiftDriveMotorUp;
                }
                break;

            case LiftDriveMotorUp:
                if (frontLiftRaised.get()) {
                    frontLift.set(frontLiftSpeedUp);
                } else {
                    // System.out.println("Finished");
                    frontLift.set(0);
                    // doManualClimb = false;
                    manualClimbState = manualClimb.MoveFullyForward;
                }

                break;

            case MoveFullyForward:
                base.TeleopMove();
                System.out.println("Moving");
                break;

            }
        }

        // -----------------------------AUTOCLIMB
        // CODE---------------------------------------------------------

        if (controls.autoClimb() && wasAutoButtonPressed == false) {
            if (doAutoClimb == true) {
                doAutoClimb = false;
                liftDrive.setEncoderToZero();
            } else {
                doAutoClimb = true;
                System.out.println("Auto Climbing Up----------------------------------------");
                autoClimbState = autoClimb.PullRobotUp;
            }
            wasAutoButtonPressed = true;
        } else if (!controls.autoClimb()) {
            wasAutoButtonPressed = false;
        }

        if (doAutoClimb) {
            leftDistance = arduino.getSensor1Data() / mmToIn;
            rightDistance = arduino.getSensor2Data() / mmToIn;

            switch (autoClimbState) {
            case PullRobotUp:
                if (Math.abs(leftDistance - rightDistance) < 1 && leftDistance <= maxDistanceToPlatform
                        && rightDistance <= maxDistanceToPlatform) {
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
                        autoClimbState = autoClimb.MoveForward;
                    }
                }
                break;

            case MoveForward:
                // if (Math.abs(liftDrive.getSensorPosition()) <= liftDriveEncodersPerRev * 20)
                // {
                if (driveSwitch.get()) {
                    base.elevatorClimb(liftDriveSpeed, liftDriveEncodersPerRev * 20);
                    liftDrive.set(liftDriveSpeed);
                    System.out.println(liftDrive.getSensorPosition());
                } else {
                    liftDrive.set(0);
                    base.stopMoving();
                    System.out.println("Auto Lifting The Drive Motor Up");
                    autoClimbState = autoClimb.LiftDriveMotorUp;
                }
                break;

            case LiftDriveMotorUp:
                if (frontLiftRaised.get()) {
                    frontLift.set(frontLiftSpeedUp);
                } else {
                    // System.out.println("Finished");
                    System.out.println("Moving Forward Again");
                    frontLift.set(0);
                    // doAutoClimb = false;
                    autoClimbState = autoClimb.MoveFullyForward;
                }
                break;

            case MoveFullyForward:

                // if(Math.abs(base.platformEncoderLeft()) <= driveEncodersPerRev){
                if (driveSwitch.get()) {
                    base.elevatorClimb(.3, driveEncodersPerRev);
                    System.out.println("Moving");
                }

                else {
                    base.stopMoving();
                    System.out.println("Finished");
                    doAutoClimb = false;
                }

                break;

            }

        }

    }

    public void raiseElevator() {
        // System.out.println("FR " + frontLiftRaised.get() + "----------------------");
        // System.out.println("FL " + frontLiftLowered.get() +
        // "----------------------");
        // System.out.println("BR " + backLiftRaised.get() + "----------------------");
        // System.out.println("BL " + backLiftLowered.get() + "----------------------");
        // System.out.println("---------------------------------------");

        SmartDashboard.putBoolean("Front Raised", frontLiftRaised.get());
        SmartDashboard.putBoolean("Front Lowered", frontLiftLowered.get());
        SmartDashboard.putBoolean("Back Raised", backLiftRaised.get());
        SmartDashboard.putBoolean("Back Lowered", backLiftLowered.get());
        SmartDashboard.putBoolean("Drive Switch", driveSwitch.get());

        if (controls.deployElevator()) {
            deploy = true;
            System.out.println("LB is Pressed");
        }

        if (deploy) {

            if (backLiftRaised.get()) {
                backLift.set(backLiftSpeedUp);
                System.out.println("Raising The Elevator");
            }

            else {
                System.out.println("Elevator is Raised");
                backLift.set(0);
                deploy = false;
            }
        }

    }

    public void testLiftDriveEncoder() {
        System.out.println(liftDrive.getSensorPosition());
    }

}