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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hatch {
    private Solenoid left;
    private Solenoid right;
    private Solenoid tilt;
    private DriverIF controls;
    // private DriveBase dBase;
    private DigitalInput leftSwitch;
    private DigitalInput rightSwitch;
    private Motor strafe;
    private Compressor compressor;
    private final double STRAFE_SPEED = .80;
    private final double VISION_LEEWAY = .4;
    private final boolean IS_COMP_BOT = true;
    private int ENCODER_COUNTS_TO_IN;
    private int ENCODER_COUNT_CENTER;
    private final double RusHatchStrafe = 0.2;
    private Vision vision;
    private Toggler autoToggle;
    private boolean firstHoming;

    public Hatch(DriverIF controls, Vision vision/* , DriveBase dBase */) {
        left = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_0);
        right = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_1);
        tilt = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_2);
        strafe = new Motor(RobotMap.ORIENTATION_MOTOR_1);
        strafe.setBrakeMode(true);
        compressor = new Compressor(RobotMap.PCM);
        leftSwitch = new DigitalInput(RobotMap.DIGITAL_INPUT_0);
        rightSwitch = new DigitalInput(RobotMap.DIGITAL_INPUT_1);
        this.vision = vision;
        this.controls = controls;
        // this.dBase = dBase;
        if (IS_COMP_BOT) {
            this.ENCODER_COUNTS_TO_IN = RobotMap.ENCODER_COUNTS_PER_IN_RUSS;
            this.ENCODER_COUNT_CENTER = RobotMap.ENCODER_COUNT_TOTAL_RUSS / 2;
        } else {
            this.ENCODER_COUNTS_TO_IN = RobotMap.ENCODER_COUNTS_TO_IN_MULE;
            this.ENCODER_COUNT_CENTER = RobotMap.ENCODER_COUNT_CENTER_MULE;
        }
        autoToggle = new Toggler(2, true);

    }

    /**
     * Method called in teleop of Robot, runs the hatch mechanism through controller
     * input
     */

    public void extend() {
        left.set(true);
        right.set(true);

    }

    public void retract() {
        left.set(false);
        right.set(false);
    }

    public void teleopInit() {
        firstHoming = true;
        System.out.println("Hatch teleopInit");
        compressor.setClosedLoopControl(true);
        // home.start();
        tilt.set(true);
    }

    public void teleop() {
        // System.out.println(strafe.getSensorPosition());
        tilt.set(controls.elevatorToggle());
        SmartDashboard.putNumber("Strafe Encoders", strafe.getSensorPosition());
        if (controls.hatchExtend()) {
            extend();
        } else {
            retract();
        }

        compressor.setClosedLoopControl(true);
        hatchStrafe();
    
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
    public void compressorOff(){
        compressor.stop();
    }
    public void compressorOn(){
        compressor.start();                                                                                                                                                             Z
    }

    public double getOffset() {
        return Math.abs((strafe.getSensorPosition() / ENCODER_COUNTS_TO_IN) - vision.getStrafeFromTarget());
    }

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
    }

    // Auto Commands

    /**
     * Freakin' cool method to automatically align the hatch to the station with
     * vision. Welcome to the future.
     */
    public class Home extends Command {
        private boolean homing;
        private boolean zeroed;
        private boolean complete;
        private boolean waiting;
        private boolean postWait;
        private double waitTime;

        public Home() {
        }

        @Override
        protected void initialize() {
            zeroed = false;
            complete = false;
            homing = false;
            waiting = false;
            postWait = false;
            waitTime = 0;
        }

        @Override
        protected void execute() {

            if (firstHoming) {

                if (!homing) {
                    homing = true;
                    strafe.set(STRAFE_SPEED);
                    System.out.println("Homing" + strafe.getSensorPosition());
                } else if (homing && !zeroed) {
                    strafe.set(STRAFE_SPEED);
                    System.out.println("Homing" + strafe.getSensorPosition());
                }
                if (!rightSwitch.get() && !zeroed) {
                    System.out.println("At right boundary");
                    zeroed = true;
                    waiting = true;
                    homing = false;
                    strafe.setEncoderToZero();
                    strafe.set(0);
                    waitTime = Timer.getFPGATimestamp();
                }
                if (waiting) {
                    System.out.println("Waiting: " + (Timer.getFPGATimestamp() - waitTime));
                    if (Timer.getFPGATimestamp() - waitTime > 1.0) {
                        System.out.println("Ending wait");
                        waiting = false;
                        postWait = true;
                        strafe.set(-STRAFE_SPEED);
                    }
                } else if (postWait) {
                    if (Math.abs(strafe.getSensorPosition() - ENCODER_COUNT_CENTER) < 100 && !waiting && zeroed) {
                        System.out.println("At center");
                        strafe.set(0);
                        complete = true;
                        firstHoming = false;
                    } else {
                        strafe.set(-STRAFE_SPEED);
                    }
                }

            } else {
                if (Math.abs(strafe.getSensorPosition() - ENCODER_COUNT_CENTER) < 100) {
                    System.out.println("At center");
                    strafe.set(0);
                    complete = true;
                } else if ((strafe.getSensorPosition() - ENCODER_COUNT_CENTER) > 0) {
                    strafe.set(STRAFE_SPEED);
                } else {
                    strafe.set(-STRAFE_SPEED);
                }
            }

        }

        @Override
        protected boolean isFinished() {
            return complete;
        }
    }

    public HatchMoveSnapshot getHatchMoveSnapshot() {
        return new HatchMoveSnapshot();
    }

    /**
     * Command written to move hatch autonomously based on a distance in inches,
     * which is converted to encoder counts. This command CAN be run parallel with a
     * drive command because it takes an initial snapshot of the vision offset, and
     * runs the hatch motor until that number of encoder counts is reached.
     */
    public class HatchMoveSnapshot extends Command {
        private double inchesToMove;
        private boolean movingHatchMechanism = false;
        private double previousPosition = 0;
        private double movedInches = 0;
        private boolean finished = false;

        public HatchMoveSnapshot() {
            System.out.println("Constructing a hatchMove");
        }

        @Override
        protected void initialize() {
            System.out.println("Initializing HatchMove");
            this.inchesToMove = vision.getDistanceFromTarget();
            this.movingHatchMechanism = false;
            previousPosition = 0;
            movedInches = 0;
            finished = false;
        }

        @Override
        protected void execute() {
            // System.out.println("Executing Hatch");
            if (vision.getDistanceFromTarget() == vision.DEFAULT_VALUE) {
                System.out.println("No data");
                this.finished = true;
            }
            if (!this.movingHatchMechanism) {
                this.movingHatchMechanism = true;
                this.finished = false;
                this.previousPosition = strafe.getSensorPosition();
                System.out.println("Starting auto alignment");
                System.out.println(vision.getDistanceFromTarget());
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

    /**
     * Command written to autonomously move hatch mechanism based on the current
     * state of vision. Not possible to be run parallel with a drive command because
     * we lose vision after 18 inches.
     */
    public HatchMoveCurrent hatchMoveCurrent(boolean cancelParent) {
        return new HatchMoveCurrent(cancelParent);
    }

    public class HatchMoveCurrent extends Command {
        private boolean finished = false;
        private boolean running = false;
        private boolean atBoundary = false;
        private boolean cancelParent;

        public HatchMoveCurrent(boolean cancelParent) {
            this.cancelParent = cancelParent;
            System.out.println("Constructing a hatchMove");
        }

        @Override
        protected void initialize() {
            System.out.println("Initializing HatchMove");
            finished = false;
            atBoundary = false;
        }

        @Override
        protected void execute() {
            double horzDistFromTarget = vision.getStrafeFromTarget();
            if (horzDistFromTarget == vision.DEFAULT_VALUE) {
                finished = true;
                System.out.println("La vision est borkeed");
            } else {
                System.out.println(horzDistFromTarget);
                System.out.println("Strafe encoders: " + strafe.getSensorPosition());
                if (Math.abs(horzDistFromTarget) < VISION_LEEWAY) {
                    System.out.println("On target");
                    strafe.set(0);
                    this.finished = true;
                } else if (horzDistFromTarget < 0 && leftSwitch.get()) {
                    strafe.set(-STRAFE_SPEED * Math.abs(horzDistFromTarget / 4) - 0.2);
                } else if (horzDistFromTarget > 0 && rightSwitch.get()) {
                    strafe.set(STRAFE_SPEED * Math.abs(horzDistFromTarget / 4) + 0.2);
                }
                if (horzDistFromTarget < 0 && !leftSwitch.get()) {
                    this.atBoundary = true;
                    this.finished = true;
                    System.out.println("At left boundary");
                    if (cancelParent) {
                        this.getGroup().cancel();
                    }
                    System.out.println("Cancelling due to being at left bound");
                } else if (horzDistFromTarget > 0 && !rightSwitch.get()) {
                    this.atBoundary = true;
                    this.finished = true;
                    System.out.println("At right boundary");
                    if (cancelParent) {
                        this.getGroup().cancel();
                    }
                    System.out.println("Cancelling due to being at right bound");
                }

            }

        }

        public boolean getAtBoundary() {
            return this.atBoundary;
        }

        @Override
        protected boolean isFinished() {
            return this.finished;
        }
    }

    /**
     * Command written to autonomously deploy the hatch and retract after a
     * specified period of time
     */
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

        @Override
        protected void execute() {
            left.set(true);
            right.set(true);
        }

        @Override
        protected void end() {
            left.set(false);
            right.set(false);
        }

    }

    // Getter methods for commands
    public HatchDeploy hatchDeploy(double time) {
        return new HatchDeploy(time);
    }

    public Home getHome() {
        return new Home();
    }

    // public HatchMove getHatchMove(double inchesToMove) {
    // return new HatchMove();
    // }
}