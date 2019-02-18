
package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.command.CommandGroup;
=======
import edu.wpi.first.wpilibj.SerialPort;
>>>>>>> 472bbda21d1af78585c0a7fbef5f7ed13dfb7201
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

    private SerialDataHandler serial = new SerialDataHandler(9600, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone,
            SerialPort.StopBits.kOne);

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
    private double frontLiftSpeedDown = -0.5;
    private double backLiftSpeedUp = 0.5;
    private double backLiftSpeedDown = -0.5;
    private double liftDriveSpeed = 0.2;

    private double closedLoopSpeed = 0;

    private double startTime;
    private double distance1ToTheWall;
    private double distance2ToTheWall;

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
        PullRobotUp, MoveForward, LiftDriveMotorUp, MoveFullyForward, Error
    }

    public Elevator(DriveBase driver, DriverIF controls) {
        base = driver;
        frontLift = new Motor(RobotMap.ACTION_MOTOR_1);
        frontLift.invert(true);
        backLift = new Motor(RobotMap.ACTION_MOTOR_2);
        liftDrive = new Motor(RobotMap.ACTION_MOTOR_3);
        elevatorDeployMotor = new Motor(RobotMap.ACTION_MOTOR_4);
        this.controls = controls;
<<<<<<< HEAD
        SmartDashboard.putData("lift front", liftElevator(0.5, 0.0));
        SmartDashboard.putData("lower front", liftElevator(-0.5, 0.0));
        SmartDashboard.putData("lift back", liftElevator(0, 0.5));
        SmartDashboard.putData("lower back", liftElevator(0, -0.5));

        SmartDashboard.putData("raise bot", liftElevator(0.5, 0.5));

        SmartDashboard.putData("drive robot", base.driveToInch(8, 0.4));
        SmartDashboard.putData("drive elevator", driveElevator(10, 0.2));
        SmartDashboard.putData("deploy elevator", deployElevator(true));
        SmartDashboard.putData("retract elevator", deployElevator(false));
=======

        frontLift.invert(true);

        frontLift.setBrakeMode(true);
        backLift.setBrakeMode(true);
        liftDrive.setBrakeMode(true);
        elevatorDeployMotor.setBrakeMode(true);
>>>>>>> 472bbda21d1af78585c0a7fbef5f7ed13dfb7201
    }


    /*
    public void teleopRaise() {
        SmartDashboard.putBoolean("Front Raised", frontLiftRaised.get());
        SmartDashboard.putBoolean("Front Lowered", frontLiftLowered.get());
        SmartDashboard.putBoolean("Back Raised", backLiftRaised.get());
        SmartDashboard.putBoolean("Back Lowered", backLiftLowered.get());
        SmartDashboard.putBoolean("Elevator Deployed", elevatorDeploy.get());

        raiseElevator();
<<<<<<< HEAD
        
=======

>>>>>>> 472bbda21d1af78585c0a7fbef5f7ed13dfb7201
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
                elevatorDeployMotor.set(0);

                if (frontLiftLowered.get()) {
                    frontLift.set(frontLiftSpeedDown);
                    System.out.println("Lowering Front");
                } else {
                    frontLift.set(0);
                    System.out.println("frontlift.set(0)");
                }

                if (backLiftLowered.get()) {
                    backLift.set(backLiftSpeedDown);
                    System.out.println("Lowering Back");
                } else {
                    backLift.set(0);
                    System.out.println("backlift.set(0)");
                }

                if (!frontLiftLowered.get() && !backLiftLowered.get()) {
                    // distance1ToTheWall = sensor1InInches();
                    // distance2ToTheWall = sensor2InInches();
                    climbState = climb.MoveForward;
                    System.out.println("Moving Forward");
                } else if (Timer.getFPGATimestamp() - startTime >= 10.0) {
                    frontLift.set(0);
                    backLift.set(0);
                    System.out.println("Timeout");
                    climbState = climb.Error;
                }

                break;

            case MoveForward:
                if (doManualClimb) {
                    liftDrive.set(-controls.throttle() * 0.25);
                    base.TeleopMove();
                    if (controls.retractLiftDrive()) {
                        System.out.println("Lifting Drive Motor Up-------------------------------------");
                        climbState = climb.LiftDriveMotorUp;
                    }
                } else {
                    // if (Math.abs(liftDrive.getSensorPosition()) <= liftDriveEncodersPerRev * 20)
                    // {
                    if ((distance1ToTheWall - sensor1InInches() < 20)
                            || (distance2ToTheWall - sensor2InInches() < 20)) {
                        liftDrive.set(.2);
                        base.moveDriveBaseElevator();
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
                    // distance1ToTheWall = sensor1InInches();
                    // distance2ToTheWall = sensor2InInches();
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

            case Error:

                break;
            }
        }

    }

    public void raiseElevator() {
        SmartDashboard.putNumber("Elevator Deploy Current", elevatorDeployMotor.getMotorCurrent());
        SmartDashboard.putBoolean("Stall Current Activity", isStallCurrentActive);
        SmartDashboard.putNumber("Back Elevator Current", backLift.getMotorCurrent());

        if (controls.deployElevator() && !deploy) {
            deploy = true;
        } else if (controls.deployElevator() && deploy) {
            deploy = false;
        }

        if (deploy) {

            if (elevatorDeployMotor.getMotorCurrent() >= 3.5) {
                isStallCurrentActive = true;

            }

            if (isStallCurrentActive) {
                elevatorDeployMotor.set(0.15);
                // deploy = false;
                System.out.println("Stall Current is on");
            }

            else {
                // if (!elevatorDeploy.get()) {
                elevatorDeployMotor.set(0.3);
                System.out.println("Stall Current is off");
            }

            if (backLiftRaised.get()) {
                backLift.set(0.4);
            } else {
                backLift.set(0);
            }

            if (!backLiftRaised.get() && isStallCurrentActive) {
                deploy = false;
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
    }//*/
    public ElevatorGroup elevatorGroup() {
        return elevatorGroup();
    }
    public class ElevatorGroup extends CommandGroup {
        public ElevatorGroup() {
            this.addSequential(deployElevator(true), 3.0);
            this.addSequential(liftElevator(0.5, 0), 4.5);
            this.addSequential(base.driveToInch(20, 0.2), 8.0);
            this.addSequential(liftElevator(-0.5, -0.5), 6.0);
            this.addSequential(driveElevator(20, 0.2), 5.0);
            this.addSequential(liftElevator(0, 0.5), 5.0);
            this.addSequential(base.driveToInch(20, 0.2), 8.0);

        }
    }
    public DriveElevator driveElevator(double inches, double speed) {
        return new DriveElevator(inches, speed);
    }
    public class DriveElevator extends CommandGroup {
        public DriveElevator(double inches, double speed) {
            this.addSequential(liftDrive.moveToEncoder(35.9 * inches, speed));
        }
    }
    public LiftElevator liftElevator(double speedFront, double speedBack) {
        return new LiftElevator(speedFront, speedBack);
    }
    public class LiftElevator extends Command {
        double speedFront, speedBack;
        private boolean frontComplete;
        private boolean backComplete;
        private DigitalInput stopSwitchFront;
        private DigitalInput stopSwitchBack;
        public LiftElevator(double speedFront, double speedBack) {
            this.speedBack = speedBack;
            this.speedFront = speedFront;
            frontComplete = false;
            backComplete = false;
        }
        @Override
        public void initialize() {
            if(speedFront > 0) {stopSwitchFront = frontLiftRaised;}
            else {stopSwitchFront = frontLiftLowered;}
            if(speedBack > 0) {stopSwitchBack = backLiftRaised;}
            else {stopSwitchBack = backLiftLowered;}
            if(speedFront == 0) frontComplete = true;
            if(speedBack == 0) backComplete = true;
        }
        public void execute() {
            if(!frontComplete) {
                if(stopSwitchFront.get()) {
                    frontComplete = true;
                    frontLift.set(0);
                } else {
                    frontLift.set(speedFront);
                }
            }
            if(!backComplete) {
                if(stopSwitchBack.get()) {
                    backComplete = true;
                    backLift.set(0);
                } else {
                    backLift.set(speedBack);
                }
            }
        }
        @Override
        protected boolean isFinished() {
            return frontComplete && backComplete;
        }
    }
    public DeployElevator deployElevator(boolean up) {
        return new DeployElevator(up);
    }
    public class DeployElevator extends Command {
        private boolean complete = false;
        private boolean up;
        public DeployElevator(boolean up) {
            this.up = up;
        }
        public void execute() {
            if(up) {
                if(elevatorDeployMotor.getMotorCurrent() >= 3.5) {
                    complete = true;
                    elevatorDeployMotor.set(0.15);
                }
                else if(!complete) {
                    elevatorDeployMotor.set(0.3);
                }
            }
            else {
                complete = true;
                elevatorDeployMotor.set(0);
            }
        }
        @Override
        protected boolean isFinished() {
            return complete;
        }
        @Override
        public void end() {

<<<<<<< HEAD
        }
    }
=======
    private double sensor1InInches() {
        return serial.getSensor1Data() * mmToIn;
    }

    private double sensor2InInches() {
        return serial.getSensor2Data() * mmToIn;
    }

>>>>>>> 472bbda21d1af78585c0a7fbef5f7ed13dfb7201
}
