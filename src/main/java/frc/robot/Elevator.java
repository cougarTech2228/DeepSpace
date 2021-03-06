
package frc.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

    private DriverIF controls;
    private DriveBase base;

    private SerialDataHandler serial = new SerialDataHandler(9600, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone,
            SerialPort.StopBits.kOne);

    // Lift Drive encoder math
    private double circumference = 4 * Math.PI;
    private int gearRatio = 63;
    private double liftDriveMovePerRev = 0.1994;
    private double liftDriveEncodersPerRev = circumference * gearRatio / liftDriveMovePerRev;

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
    private double backElevatorEncodersPerInch = 0;
    private double frontElevatorEncodersPerInch = 0;

    private int climbLevel = 0;

    private Toggler TeleClimb = new Toggler(5);
    private CommandGroup[] climbSequence = new CommandGroup[4];

    public Elevator(DriveBase driver, DriverIF controls) {
        base = driver;
        frontLift = new Motor(RobotMap.ACTION_MOTOR_1);
        // frontLift.invert(true);
        backLift = new Motor(RobotMap.ACTION_MOTOR_2);
        liftDrive = new Motor(RobotMap.ACTION_MOTOR_3);
        liftDrive.invert(true);
        elevatorDeployMotor = new Motor(RobotMap.ACTION_MOTOR_4);
        this.controls = controls;
        // SmartDashboard.putData("lift front", liftElevator(0.5, 0.0, false));
        // SmartDashboard.putData("lower front", liftElevator(-0.5, 0.0, false));
        // SmartDashboard.putData("lift back", liftElevator(0, 0.5, false));
        // SmartDashboard.putData("lower back", liftElevator(0, -0.5, false));

        frontLift.setBrakeMode(true);
        backLift.setBrakeMode(true);
        liftDrive.setBrakeMode(true);
        elevatorDeployMotor.setBrakeMode(true);

        // SmartDashboard.putData("raise bot", raise);

        SmartDashboard.putData("drive robot", base.driveToInch(8, 0.4));
        SmartDashboard.putData("drive elevator", driveElevator(10, 0.2));
        SmartDashboard.putData("deploy elevator", deployElevator(true));
        SmartDashboard.putData("retract elevator", deployElevator(false));
    }

    public void updateSwitches() {
        SmartDashboard.putBoolean("Front Raised", frontLiftRaised.get());
        SmartDashboard.putBoolean("Front Lowered", frontLiftLowered.get());
        SmartDashboard.putBoolean("Back Raised", backLiftRaised.get());
        SmartDashboard.putBoolean("Back Lowered", backLiftLowered.get());
        SmartDashboard.putBoolean("Elevator Deployed", elevatorDeploy.get());
    }

    public void teleopInit() {
        elevatorDeployMotor.set(0);
        frontLift.set(0);
        backLift.set(0);
        liftDrive.set(0);
        Scheduler.getInstance().removeAll();
        for (int i = 0; i < climbSequence.length; i++) {
            climbSequence[i] = new CommandGroup();
        }
    }

    public void teleopPeriodic() {
        if(controls.manualOverride()) {
            elevatorDeployMotor.set(.4);
        }
        else {
            elevatorDeployMotor.set(0);
        }

        if (controls.climb3ndLvl() && climbLevel == 0) {
            climbLevel = 3;
        } else if (controls.climb2ndLvl() && climbLevel == 0) {
            climbLevel = 2;
        }
        

        int num = TeleClimb.state;
        TeleClimb.toggle(controls.climb2ndLvl() || controls.climb3ndLvl());

        if (TeleClimb.state >= 1 && TeleClimb.state <= 2 && climbLevel == 3) {

            liftDrive.set(controls.throttle() * 1.3);
        }
        if (TeleClimb.state >= 1 && TeleClimb.state <= 3 && climbLevel == 2) {
            liftDrive.set(controls.throttle() * 1.3);
        }
        if (num != TeleClimb.state) {
            switch (TeleClimb.state) {
            case 1: {

                if (climbLevel == 3) {
                    climbSequence[0].addSequential(liftElevator(0, 1, 0), 4.5);
                    //climbSequence[0].addSequential(deployElevator(true), 3.0);
                } else if (climbLevel == 2) {
                    climbSequence[0].addSequential(liftElevator(0, 1, 0), 4.5);
                    //climbSequence[0].addSequential(deployElevator(true), 3.0);
                }
                climbSequence[0].start();
                base.setMaxSpeed(0.5);
            }
                break;
            case 2: {
                climbSequence[0].cancel();
                if (climbLevel == 3) {
                    climbSequence[1].addSequential(liftElevator(-0.9, -0.5, .20), 12.0);
                } else if (climbLevel == 2) {
                    climbSequence[1].addSequential(liftElevator(0, -0.75, .25), 4.0);
                    climbSequence[1].addSequential(liftElevator(-0.9, 0, 0.1), 4.0);
                }
                climbSequence[1].start();

            }
                break;
            case 3: {
                elevatorDeployMotor.set(0);
                climbSequence[1].cancel();
                if (climbLevel == 3) {
                    liftDrive.set(0);
                    climbSequence[2].addSequential(liftElevator(1, 0, 0));
                    climbSequence[2].start();
                }
                base.setMaxSpeed(1);
            }
                break;
            case 4: {
                if (climbLevel == 2) {
                    climbSequence[2].addSequential(liftElevator(1, 0, 0));
                    climbSequence[2].start();
                }
                climbSequence[2].cancel();

                frontLift.set(0);
                liftDrive.set(0);

            }
                break;
            }
        }
    }

    public DriveElevator driveElevator(double time, double speed) {
        return new DriveElevator(time, speed);
    }

    public class DriveElevator extends Command {
        double startTime;
        double totalTime;
        double speed;

        public DriveElevator(double time, double speed) {
            liftDrive.set(speed);
            totalTime = time;
            this.speed = speed;
        }

        @Override
        public void execute() {
        }

        @Override
        protected void initialize() {
            startTime = Timer.getFPGATimestamp();
        }

        @Override
        protected boolean isFinished() {
            return (startTime + totalTime) >= Timer.getFPGATimestamp();
        }

        @Override
        public void end() {
            liftDrive.set(0);
        }
    }

    public LiftElevator liftElevator(double speedFront, double speedBack, double stallCurrent) {
        return new LiftElevator(speedFront, speedBack, stallCurrent);
    }

    public class LiftElevator extends Command {
        double speedFront, speedBack, frontEncoder, backEncoder;
        private boolean frontComplete;
        private boolean backComplete;
        private double stallCurrent = 0;
        private DigitalInput stopSwitchFront;
        private DigitalInput stopSwitchBack;

        public LiftElevator(double speedFront, double speedBack, double frontEncoderDistance,
                double backEncoderDistance) {
            this.speedBack = speedBack;
            this.speedFront = speedFront;
            frontEncoder = frontEncoderDistance;
            backEncoder = backEncoderDistance;
        }

        public LiftElevator(double speedFront, double speedBack, double stallCurrent) {
            this.speedBack = speedBack;
            this.speedFront = speedFront;
            this.stallCurrent = stallCurrent;
        }

        @Override
        public void initialize() {
            frontComplete = false;
            backComplete = false;
            if (speedFront > 0) {
                stopSwitchFront = frontLiftRaised;
            } else {
                stopSwitchFront = frontLiftLowered;
            }

            if (speedBack > 0) {
                stopSwitchBack = backLiftRaised;
            } else {
                stopSwitchBack = backLiftLowered;
            }

            if (speedFront == 0)
                frontComplete = true;
            if (speedBack == 0)
                backComplete = true;
        }

        @Override
        public void execute() {
            SmartDashboard.putBoolean("Front Complete", frontComplete);
            SmartDashboard.putBoolean("Back Complete", backComplete);

            if (!stopSwitchFront.get() || frontComplete) {
                frontComplete = true;
                frontLift.set(-stallCurrent * 1.5);
            } else {
                // System.out.printlnln("Running Front");
                frontLift.set(speedFront);
            }

            if (!stopSwitchBack.get() || backComplete) {
                backComplete = true;
                backLift.set(-stallCurrent);
            } else {
                // System.out.printlnln("Running Back");
                backLift.set(speedBack);
            }
        }

        @Override
        protected boolean isFinished() {
            return (frontComplete && backComplete);
        }

        @Override
        public void end() {
            super.end();
            backLift.set(0);
            frontLift.set(0);
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
            if (up) {
                if (elevatorDeployMotor.getMotorCurrent() >= 9.0) {
                    complete = true;
                    elevatorDeployMotor.set(0.20);
                } else if (!complete) {
                    elevatorDeployMotor.set(0.5);

                }
            } else {
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

        }
    }

    private double sensor1InInches() {
        return serial.getSensor1Data() * mmToIn;
    }

    private double sensor2InInches() {
        return serial.getSensor2Data() * mmToIn;
    }

    public void putElevatorEncoders() {
        SmartDashboard.putNumber("Front Elevator Encoders", frontLift.getSensorPosition());
        SmartDashboard.putNumber("Back Elevator Encoders", backLift.getSensorPosition());
    }

    public void runLiftDrive() {
        liftDrive.set(controls.strafe());
    }
}
