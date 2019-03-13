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
    private Solenoid top;
    private Solenoid bottom;
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
    private int ENCODER_COUNT_TOTAL;
    private final double RusHatchStrafe = 0.2;
    private Vision vision;
    private Toggler autoToggle;
    private boolean firstHoming;

    public Hatch(DriverIF controls, Vision vision/* , DriveBase dBase */) {
        top = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_0);
        bottom = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_1);
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
            this.ENCODER_COUNT_TOTAL = RobotMap.ENCODER_COUNT_TOTAL_RUSS;
        } else {
            this.ENCODER_COUNTS_TO_IN = RobotMap.ENCODER_COUNTS_TO_IN_MULE;
            this.ENCODER_COUNT_CENTER = RobotMap.ENCODER_COUNT_CENTER_MULE;
        }
        firstHoming = true;
        autoToggle = new Toggler(2, true);

    }

    /**
     * Method called in teleop of Robot, runs the hatch mechanism through controller
     * input
     */

    public void extend() {
        top.set(true);
        bottom.set(true);

    }

    public void retract() {
        top.set(false);
        bottom.set(false);
    }

    public void teleopInit() {
        firstHoming = true;
        //System.out.printlnln("Hatch teleopInit");
        compressor.setClosedLoopControl(true);
        Home tempHome = new Home();
        tempHome.start();
        tilt.set(true);
    }

    public void teleop() {
        // //System.out.printlnln(strafe.getSensorPosition());
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
            //System.out.printlnln("Hatch Motor: " + strafe.getSensorPosition());
        } else {
            strafe.setSpeed(0);
            strafe.setEncoderToZero();
        }
    }
    // public void compressorOff(){
    // compressor.stop();
    // }
    // public void compressorOn(){
    // compressor.start(); Z
    // }

    public boolean getOffset(double inToMove) {
        // return Math.abs((strafe.getSensorPosition() / ENCODER_COUNTS_TO_IN) - vision.getStrafeFromTarget());
        if(inToMove < 0){
            if((inToMove * ENCODER_COUNTS_TO_IN) + strafe.getSensorPosition() < this.ENCODER_COUNT_TOTAL){
                return true;
            }
        }
        else{
            if(strafe.getSensorPosition() - (inToMove * ENCODER_COUNTS_TO_IN) > 0){
                return true;
            }
        }
        return false;
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
                // //System.out.printlnln("This is first homing");
                if (!homing) {
                    homing = true;
                    strafe.set(STRAFE_SPEED / 1.5);
                    // //System.out.printlnln("Homing" + strafe.getSensorPosition());
                } else if (homing && !zeroed) {
                    strafe.set(STRAFE_SPEED / 1.5);
                    // //System.out.printlnln("Homing" + strafe.getSensorPosition());
                }
                if (!rightSwitch.get() && !zeroed) {
                    // //System.out.printlnln("At right boundary");
                    zeroed = true;
                    waiting = true;
                    homing = false;
                    strafe.setEncoderToZero();
                    strafe.set(0);
                    waitTime = Timer.getFPGATimestamp();
                }
                if (waiting) {
                    // //System.out.printlnln("Waiting: " + (Timer.getFPGATimestamp() - waitTime));
                    if (Timer.getFPGATimestamp() - waitTime > .3) {
                        //System.out.printlnln("Ending wait");
                        waiting = false;
                        postWait = true;
                        strafe.set(-STRAFE_SPEED / 1.5);
                    }
                } else if (postWait) {
                    if (strafe.getSensorPosition() > ENCODER_COUNT_CENTER && !waiting && zeroed) {
                        // //System.out.printlnln("At center");
                        strafe.set(0);
                        complete = true;
                        firstHoming = false;
                    } else {
                        strafe.set(-STRAFE_SPEED / 1.5);
                        // //System.out.printlnln("Encoder cts: " + strafe.getSensorPosition());

                    }
                }

            } else {
                if (Math.abs(strafe.getSensorPosition() - ENCODER_COUNT_CENTER) < 10000) {
                    // //System.out.printlnln("At center");
                    strafe.set(0);
                    complete = true;
                } else if ((strafe.getSensorPosition() - ENCODER_COUNT_CENTER) > 0) {
                    // //System.out.printlnln("Encoder cts: " + strafe.getSensorPosition());
                    strafe.set(STRAFE_SPEED / 1.5);
                } else {
                    // //System.out.printlnln("Encoder cts: " + strafe.getSensorPosition());
                    strafe.set(-STRAFE_SPEED / 1.5);
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
        private double visionInches;
        private double encoderCtsToMove;
        private boolean movingHatchMechanism = false;
        private double previousPosition;
        private double movedCts = 0;
        private final double AUTO_HATCH_FLOOR = .075;
        private final double AUTO_STRAFE_SPEED = STRAFE_SPEED - AUTO_HATCH_FLOOR;
        private double moveSpeed = 0;
        private boolean finished = false;
        private boolean movingLeft, movingRight;

        public HatchMoveSnapshot() {
            //System.out.printlnln("Constructing a hatchMove");
            this.visionInches = vision.getStrafeFromTarget();
            this.encoderCtsToMove = (this.visionInches * ENCODER_COUNTS_TO_IN);
            this.previousPosition = strafe.getSensorPosition();
            this.movingHatchMechanism = false;
            movedCts = 0;
            moveSpeed = STRAFE_SPEED;
            finished = false;
            movingLeft = false;
            movingRight = false;
        }

        @Override
        protected void initialize() {
            //System.out.printlnln("Initializing HatchMove");
            this.visionInches = vision.getStrafeFromTarget();
            this.encoderCtsToMove = (this.visionInches * ENCODER_COUNTS_TO_IN);
            this.previousPosition = strafe.getSensorPosition();
            this.movingHatchMechanism = false;
            movedCts = 0;
            moveSpeed = STRAFE_SPEED;
            finished = false;
            movingLeft = false;
            movingRight = false;
        }

        @Override
        protected void execute() {
            // //System.out.printlnln("Executing Hatch");
            if (vision.getDistanceFromTarget() == vision.DEFAULT_VALUE) {
                //System.out.printlnln("No data");
                this.getGroup().cancel();
                this.finished = true;
            }
            if (!this.movingHatchMechanism) {
                this.movingHatchMechanism = true;
                this.finished = false;
                // //System.out.printlnln("Starting auto alignment");
                // //System.out.printlnln("Starting Position: " + this.previousPosition);
                // //System.out.printlnln("Inches away from center: " + this.encoderCtsToMove / ENCODER_COUNTS_TO_IN);

                // //System.out.printlnln("Encoder Counts to move" + this.encoderCtsToMove);
                if (this.visionInches < 0 && leftSwitch.get()) {
                    strafe.set(-STRAFE_SPEED);
                    movingLeft = true;
                    movingRight = false;
                    //System.out.printlnln("moving left");
                } else if (this.visionInches > 0 && rightSwitch.get()) {
                    strafe.set(STRAFE_SPEED);
                    movingLeft = false;
                    movingRight = true;
                    //System.out.printlnln("Moving right");
                }
                this.encoderCtsToMove = Math.abs(encoderCtsToMove);

            } else if (this.movingHatchMechanism) {
                if (movingLeft) {
                    this.movedCts = (strafe.getSensorPosition() - this.previousPosition);
                } else if (movingRight) {
                    this.movedCts = (this.previousPosition - strafe.getSensorPosition());
                }
                moveSpeed = (AUTO_STRAFE_SPEED - (Math.abs(this.movedCts / this.encoderCtsToMove) * AUTO_STRAFE_SPEED)) + AUTO_HATCH_FLOOR;
                if (this.movedCts > this.encoderCtsToMove) {
                    strafe.set(0);
                    this.finished = true;
                    //System.out.printlnln("GOT EM");
                } else if (movingRight && rightSwitch.get()) {
                    strafe.set(moveSpeed);
                    // //System.out.printlnln("moving to the right");
                } else if (movingLeft && leftSwitch.get()) {
                    strafe.set(-moveSpeed);
                    // //System.out.printlnln("moving to the left");
                } else if (strafe.getSensorPosition() < 0) {
                    strafe.set(0);
                    this.finished = true;
                    //System.out.printlnln("GOT EM");
                }
            } else {
                //System.out.printlnln("If you see this, something is super borked");
            }
        }

        @Override
        protected boolean isFinished() {
            return this.finished;
        }

    }

    // @Override
    // protected boolean isFinished() {
    // return this.finished;
    // }
    // }

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
            //System.out.printlnln("Constructing a hatchMove");
        }

        @Override
        protected void initialize() {
            //System.out.printlnln("Initializing HatchMove");
            finished = false;
            atBoundary = false;
        }

        @Override
        protected void execute() {
            double horzDistFromTarget = vision.getStrafeFromTarget();
            if (horzDistFromTarget == vision.DEFAULT_VALUE) {
                finished = true;
                //System.out.printlnln("La vision est borkeed");
            } else {
                //System.out.printlnln(horzDistFromTarget);
                //System.out.printlnln("Strafe encoders: " + strafe.getSensorPosition());
                if (Math.abs(horzDistFromTarget) < VISION_LEEWAY) {
                    //System.out.printlnln("On target");
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
                    //System.out.printlnln("At left boundary");
                    if (cancelParent) {
                        this.getGroup().cancel();
                    }
                    //System.out.printlnln("Cancelling due to being at left bound");
                } else if (horzDistFromTarget > 0 && !rightSwitch.get()) {
                    this.atBoundary = true;
                    this.finished = true;
                    //System.out.printlnln("At right boundary");
                    if (cancelParent) {
                        this.getGroup().cancel();
                    }
                    //System.out.printlnln("Cancelling due to being at right bound");
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
            //System.out.printlnln("Constructing hatchDeploy");
        }

        @Override
        protected void initialize() {
            //System.out.printlnln("Initializing HatchDeploy");
        }

        @Override
        protected boolean isFinished() {
            return isTimedOut();
        }

        @Override
        protected void execute() {
            top.set(true);
            bottom.set(true);
        }

        @Override
        protected void end() {
            top.set(false);
            bottom.set(false);
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