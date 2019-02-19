
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
    private double backElevatorEncodersPerInch = 0;
    private double frontElevatorEncodersPerInch = 0;

    private double startTime;
    private double distance1ToTheWall;
    private double distance2ToTheWall;
    private Toggler TeleClimb = new Toggler(4);
    private Toggler level2Climb = new Toggler(4);
    private CommandGroup[] climbSequence = new CommandGroup[4];

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
        //frontLift.invert(true);
        backLift = new Motor(RobotMap.ACTION_MOTOR_2);
        liftDrive = new Motor(RobotMap.ACTION_MOTOR_3);
        elevatorDeployMotor = new Motor(RobotMap.ACTION_MOTOR_4);
        this.controls = controls;
        SmartDashboard.putData("lift front", liftElevator(0.5, 0.0));
        SmartDashboard.putData("lower front", liftElevator(-0.5, 0.0));
        SmartDashboard.putData("lift back", liftElevator(0, 0.5));
        SmartDashboard.putData("lower back", liftElevator(0, -0.5));

        CommandGroup raise = new CommandGroup();
        raise.addSequential(liftElevator(-0.6, -0.5));
        raise.addParallel(driveElevator(5, 0.1));

        SmartDashboard.putData("raise bot", raise);

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
        liftDrive.set(0);
        Scheduler.getInstance().removeAll();
        for(int i = 0; i < climbSequence.length; i++) {
            climbSequence[i] = new CommandGroup();
        }
        /*
        climbSequence[0].addSequential(deployElevator(true), 3.0);
        climbSequence[0].addParallel(liftElevator(0, 0.5), 4.5);

       

        climbSequence[2].addSequential(liftElevator(0.5, 0));
        */
    }
    public void teleopPeriodic() {

        System.out.println("Driving Drive Motor: " + liftDrive.getMotorCurrent());
        System.out.println("Driving Deploy: " + elevatorDeployMotor.getMotorCurrent());
        System.out.println("Driving Front Elevator: " + frontLift.getMotorCurrent());
        System.out.println("Driving Back Elevator: " + backLift.getMotorCurrent());
        int num = TeleClimb.state;
        boolean level2 = false;
        TeleClimb.toggle(controls.manualClimb());
        TeleClimb.toggle(controls.level2Climb());
        if((num == 2 || num == 1) && controls.level2Climb()){
            level2 = true;
        }

        // if(TeleClimb.state >= 1 && TeleClimb.state <= 2) {
        if(TeleClimb.state == 2) {
            liftDrive.set(controls.throttle());
        }

        if(num != TeleClimb.state) {
            switch(TeleClimb.state) {
                case 1: {
                    climbSequence[0].addSequential(deployElevator(true), 3.0);
                    if(!level2){
                    climbSequence[0].addParallel(liftElevator(0, 0.5), 4.5);
                    }
                    else{
                        climbSequence[0].addParallel(liftElevator(0, 0.5), 1.5);
                    }
                    climbSequence[0].start();
                    base.setMaxSpeed(0.6);
                } break;
                case 2: {
                    if(!level2) {
                        climbSequence[1].addSequential(liftElevator(-0.7, -0.5), 6.0);
                        liftDrive.set(.1);
                    }
                    else{
                        climbSequence[1].addSequential(liftElevator(-0.7, -0.5), 3.0);
                    }
                    climbSequence[1].start();
                    
                } break;
                    case 3: {
                    liftDrive.set(0);
                    climbSequence[2].addSequential(liftElevator(0.5, 0));
                    climbSequence[2].start();
                    base.setMaxSpeed(1);
                } break;
            }
        }

        if(num != level2Climb.state) {
            switch(level2Climb.state) {
                case 1: {
                    climbSequence[0].start();
                    base.setMaxSpeed(0.6);
                } break;
                case 2: {
                    climbSequence[1].start();
                    liftDrive.set(0);
                } break;
                case 3: {
                    climbSequence[2].start();
                    base.setMaxSpeed(1);
                } break;
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
    public LiftElevator liftElevator(double speedFront, double speedBack) {
        return new LiftElevator(speedFront, speedBack);
    }
    public class LiftElevator extends CommandGroup {
        double speedFront, speedBack, frontEncoder, backEncoder;
        private boolean frontComplete;
        private boolean backComplete;
        private DigitalInput stopSwitchFront;
        private DigitalInput stopSwitchBack;
        public LiftElevator(double speedFront, double speedBack, double frontEncoderDistance, double backEncoderDistance){
            this.speedBack = speedBack;
            this.speedFront = speedFront;
            frontEncoder = frontEncoderDistance;
            backEncoder = backEncoderDistance;
            this.addSequential(frontLift.moveToEncoder(frontEncoder, speedFront));
            this.addParallel(backLift.moveToEncoder(backEncoder, speedBack));
        }
        public LiftElevator(double speedFront, double speedBack) {
            this.speedBack = speedBack;
            this.speedFront = speedFront;
        }
        @Override
        public void initialize() {
            frontComplete = false;
            backComplete = false;
            if(speedFront > 0) {stopSwitchFront = frontLiftRaised;}
            else {stopSwitchFront = frontLiftLowered;}

            if(speedBack > 0) {stopSwitchBack = backLiftRaised;}
            else {stopSwitchBack = backLiftLowered;}

            if(speedFront == 0) frontComplete = true;
            if(speedBack == 0) backComplete = true;
        }
        @Override
        public void execute() {
            super.execute();
            if(!frontComplete) {
                if(!stopSwitchFront.get()) {
                    frontComplete = true;
                    frontLift.set(0);
                } else {
                    frontLift.set(speedFront);
                }
            }
            if(!backComplete) {
                if(!stopSwitchBack.get()) {
                    backComplete = true;
                    backLift.set(0);
                } else {
                    backLift.set(speedBack);
                }
            }
        }
        @Override
        protected boolean isFinished() {
            return (frontComplete && backComplete) || super.isFinished();
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
                    elevatorDeployMotor.set(0.5);
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
        public void end() {}
    }


    private double sensor1InInches() {
        return serial.getSensor1Data() * mmToIn;
    }

    private double sensor2InInches() {
        return serial.getSensor2Data() * mmToIn;
    }

    public void putElevatorEncoders(){
        SmartDashboard.putNumber("Front Elevator Encoders", frontLift.getSensorPosition());
        SmartDashboard.putNumber("Back Elevator Encoders", backLift.getSensorPosition());
    }

}
