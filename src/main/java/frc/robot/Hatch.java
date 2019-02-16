package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Hatch {
    private Solenoid left;
    private Solenoid right;
    private DriverIF controls;
    private DriveBase dBase;
    private DigitalInput leftSwitch;
    private DigitalInput rightSwitch;
    private Motor strafe;
    private Compressor compressor;
    private final double STRAFE_SPEED = .35;
    private final int ENCODER_COUNTS_TO_IN = 54666;
    private final int ENCODER_COUNT_CENTER = 164000;
    private final int DEFAULT_VALUE = 42069666;

    private final String TABLE_KEY = "datatable";
    private NetworkTableInstance visionDataTableInst;
    private NetworkTable visionDataTable;
    // state, 0 for searching, 1 for acquiring, or 2 for locked
    private NetworkTableEntry targState;
    // distance to target in inches,
    private NetworkTableEntry distTargIn;
    // horizontal distance from center of target, positive being to the right
    private NetworkTableEntry horzOffToIn;
    private boolean homing = false;
    private boolean zeroed = false;
    private Toggler autoToggle;
    private int autotestingtemp = 0;
    private AutoDeploy autoDeployGroup;

    public Hatch(DriverIF controls, DriveBase dBase) {
        left = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_0);
        right = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_1);
        strafe = new Motor(RobotMap.ORIENTATION_MOTOR_1);
        strafe.setBrakeMode(true);
        compressor = new Compressor(RobotMap.PCM);
        leftSwitch = new DigitalInput(RobotMap.DIGITAL_INPUT_0);
        rightSwitch = new DigitalInput(RobotMap.DIGITAL_INPUT_1);
        this.controls = controls;
        this.dBase = dBase;
        visionDataTableInst = NetworkTableInstance.getDefault();
        visionDataTable = visionDataTableInst.getTable(TABLE_KEY);
        targState = visionDataTable.getEntry("targState");
        distTargIn = visionDataTable.getEntry("distTargetIn");
        horzOffToIn = visionDataTable.getEntry("horzOffToIn");
        this.autoDeployGroup = new AutoDeploy();
        autoToggle = new Toggler(2, true);
        int count = 0;

    }
    /**
     * Method called in teleop of Robot, runs the hatch mechanism through controller input
     */
    public void teleop() {
        autoToggle.toggle(controls.autoAlign());
        compressor.setClosedLoopControl(true);
        if (controls.hatchExtend()) {
            // extend();
            left.set(true);
            // } else if (controls.hatchExtendBottom()) {
            // right.set(true);
        } else if (controls.hatchRetract()) {
            retract();
        }
        if (autoToggle.state == 1 && !autoDeployGroup.isRunning()) {
            System.out.println("Starting auto hatch alignment from button press");
            autoDeployGroup.start();
        } else if (autoToggle.state == 0 && autoDeployGroup.isRunning()) {
            autoDeployGroup.cancel();
        }
        hatchStrafe();

        // System.out.println("distTargIn" + distTargIn.getDouble(99));
        // System.out.println("horzOffToIn" + horzOffToIn.getDouble(99));
    }
    /**
     * Method called during testPeriodic, runs hatch mechanism in test
     */
    public void testPeriodic() {
        if (Math.abs(controls.encoderTestHatch()) > 0.1) {
            strafe.setSpeed(controls.encoderTestHatch());
            System.out.println("Hatch Motor: " + strafe.getSensorPosition());
        } else {
            strafe.setSpeed(0);
            strafe.setEncoderToZero();
        }
    }
    /**
     * 
     */
    public void hatchStrafe() {
        if (!rightSwitch.get()) {
            strafe.setEncoderToZero();
        }
        if (controls.hatchStrafeLeft() && leftSwitch.get()) {
            strafe.set(-STRAFE_SPEED);
        } else if (controls.hatchStrafeRight() && rightSwitch.get()) {
            strafe.set(STRAFE_SPEED);
        } else {
            strafe.set(0);
        }
        // System.out.println(strafe.getSensorPosition());
    }

    public void extend() {
        left.set(true);
        right.set(true);

    }

    public void retract() {
        left.set(false);
        right.set(false);
    }

    public void home() {
        if (!homing) {
            strafe.set(STRAFE_SPEED);
            homing = true;
        }
        if (!rightSwitch.get() && !zeroed) {
            strafe.setEncoderToZero();
            zeroed = true;
        }
        strafe.set(-STRAFE_SPEED);
        if (Math.abs(strafe.getSensorPosition() - ENCODER_COUNT_CENTER) < 100) {
            strafe.set(0);
            homing = false;
            zeroed = false;
        }
    }

    // Auto Commands

    /**
     * Freakin' cool method to automatically align the hatch to the station with
     * vision. Welcome to the future.
     */
    public class AutoDeploy extends CommandGroup {

        public AutoDeploy() {
            this.addSequential(new HatchMove());
            this.addSequential(dBase.driveToInch(distTargIn.getDouble(0), 0.4));
            this.addSequential(new HatchDeploy(.1));
            this.addSequential(dBase.driveToInch(-3, 0.4));
        }

        @Override
        protected void initialize() {
            if (distTargIn.getDouble(DEFAULT_VALUE) > 18 && distTargIn.getDouble(DEFAULT_VALUE) < 48) {
                System.out.println(distTargIn.getDouble(DEFAULT_VALUE));
                if (targState.getDouble(DEFAULT_VALUE) == 2.0) {
                    double offset = ((strafe.getSensorPosition() / ENCODER_COUNTS_TO_IN)
                            - horzOffToIn.getDouble(DEFAULT_VALUE));
                    if (!(offset < 6 && offset > 0)) {
                        System.out.println("Offset is too great: " + offset);
                        this.cancel();
                    }
                } else {
                    System.out.println("Not locked");
                    this.cancel();
                }
            } else {
                System.out.println("Not in specified distance");
                System.out.println(distTargIn.getDouble(DEFAULT_VALUE));
                this.cancel();
            }

        }

        @Override
        protected void interrupted() {
            Scheduler.getInstance().removeAll();
        }

        @Override
        public void end() {
            autoToggle.state = 0;
            super.end();
        }
    }

    public class Home extends Command {
        private boolean homing;
        private boolean zeroed;
        private boolean complete;
        private boolean waiting;
        private double waitTime;

        public Home() {
        }

        @Override
        protected void initialize() {
            zeroed = false;
            complete = false;
            homing = false;
            waiting = false;
            waitTime = 0;
        }

        @Override
        protected void execute() {
            System.out.println("Homing" + strafe.getSensorPosition());
            if (!homing) {
                homing = true;
                strafe.set(STRAFE_SPEED);
            } else if (!rightSwitch.get() && !zeroed) {
                System.out.println("At right boundary");
                zeroed = true;
                waiting = true;
                strafe.setEncoderToZero();
                strafe.set(0);
                waitTime = Timer.getFPGATimestamp();
            } else if (waiting) {
                System.out.println("Waiting: " + (Timer.getFPGATimestamp() - waitTime));
                if (Timer.getFPGATimestamp() - waitTime > 1.0) {
                    System.out.println("Ending wait");
                    waiting = false;
                    strafe.set(-STRAFE_SPEED);
                }
            } else if (Math.abs(strafe.getSensorPosition() - ENCODER_COUNT_CENTER) < 10000 && !waiting && zeroed) {
                System.out.println("At center");
                strafe.set(0);
                complete = true;
            }
            System.out.println("Hatch mtr: " + strafe.getSensorPosition());
        }

        @Override
        protected boolean isFinished() {
            return complete;
        }
    }

    public class HatchMove extends Command {
        private double inchesToMove;
        private boolean movingHatchMechanism = false;
        private double previousPosition = 0;
        private double movedInches = 0;
        private boolean finished = false;

        public HatchMove() {
            System.out.println("Constructing a hatchMove");
        }

        @Override
        protected void initialize() {
            System.out.println("Initializing HatchMove");
            this.inchesToMove = horzOffToIn.getDouble(DEFAULT_VALUE);
            this.movingHatchMechanism = false;
            previousPosition = 0;
            movedInches = 0;
            finished = false;
        }

        @Override
        protected void execute() {
            // System.out.println("Executing Hatch");
            if (!this.movingHatchMechanism) {
                this.movingHatchMechanism = true;
                this.finished = false;
                this.previousPosition = strafe.getSensorPosition();
                System.out.println("Starting auto alignment");
                System.out.println(distTargIn.getDouble(99));
                System.out.println("Inches away from center: " + inchesToMove);
                if (inchesToMove < 0 && leftSwitch.get()) {
                    strafe.set(-STRAFE_SPEED);
                    System.out.println("moving left");
                } else if (inchesToMove > 0 && rightSwitch.get()) {
                    strafe.set(STRAFE_SPEED);
                    System.out.println("Moving right");
                } else {
                    System.out.println("wut");
                }

            } else if (this.movingHatchMechanism) {
                this.movedInches = (strafe.getSensorPosition() - this.previousPosition) / ENCODER_COUNTS_TO_IN;
                if (this.inchesToMove - this.movedInches > -.1 && this.inchesToMove - this.movedInches < .1) {
                    strafe.set(0);
                    this.movedInches = 0;
                    this.movingHatchMechanism = false;
                    this.finished = true;
                    System.out.println("GOT EM");
                } else if (this.movedInches > this.inchesToMove && rightSwitch.get()) {
                    strafe.set(STRAFE_SPEED);
                    System.out.println("moving to the right");
                } else if (this.movedInches < this.inchesToMove && leftSwitch.get()) {
                    strafe.set(-STRAFE_SPEED);
                    System.out.println("moving to the left");
                }
            } else {
                System.out.println("If you see this, something is super borked");
            }
        }

        @Override
        protected boolean isFinished() {
            return this.finished;
        }
    }

    public class HatchDeploy extends Command {
        public HatchDeploy(double time) {
            super(time);
            System.out.println("Constructing hatchDeploy");
        }

        @Override
        protected void initialize() {
            System.out.println("Initializing HatchDeploy");
        }

        @Override
        protected boolean isFinished() {
            return isTimedOut();
        }

        protected void execute() {
            left.set(true);
            right.set(true);
        }

        protected void end() {
            // left.set(false);
            // right.set(false);
        }

    }

    // Getter methods for commands
    public HatchDeploy hatchDeploy(double time) {
        return new HatchDeploy(time);
    }

    public Home getHome() {
        return new Home();
    }

    public HatchMove hatchMove(double inchesToMove) {
        return new HatchMove();
    }

}