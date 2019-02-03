package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;

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

    }

    public void extend() {
        left.set(true);
        right.set(true);
    }

    public void retract() {
        left.set(false);
        right.set(false);
    }

    public void teleop() {
        compressor.setClosedLoopControl(true);
        if (controls.hatchExtend()) {
            extend();
        } else if (controls.hatchRetract()) {
            retract();
        }
        if (controls.autoAlign()) {
            autoDeploy();
        }
        hatchStrafe();
    }

   

    public void home() {
        if (!homing) {
            strafe.set(STRAFE_SPEED);
            homing = true;
        }
        if (rightSwitch.get() && !zeroed) {
            strafe.setEncoderPosition(0);
            zeroed = true;
        }
        strafe.set(-STRAFE_SPEED);
        if (strafe.getSensorPosition() - ENCODER_COUNT_CENTER > -100
                && strafe.getSensorPosition() - ENCODER_COUNT_CENTER < 100) {
            strafe.set(0);
            homing = false;
            zeroed = false;
        }
    }

    public void hatchStrafe() {
        if (!rightSwitch.get()) {
            strafe.setEncoderPosition(0);
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

    /**
     * Freakin' cool method to automatically align the hatch to the station with
     * vision. Welcome to the future.
     */
    public void autoDeploy() {
        if (distTargIn.getDouble(99) < 24) {
            if (targState.getDouble(0) == 2.0) {
                new hatchMove(horzOffToIn.getDouble(99));
                extend();
            } else {
                System.out.println("Not locked on");
            }
        } else {
            System.out.println("Too far");
        }
    }

    public class hatchMove extends Command {
        private double inchesToMove;
        private boolean movingHatchMechanism = false;
        private double previousPosition = 0;
        private double movedInches = 0;
        private boolean finished = false;

        public hatchMove(double inchesToMove) {
            this.inchesToMove = inchesToMove;
        }

        @Override
        protected void execute() {
            if (!this.movingHatchMechanism) {
                this.movingHatchMechanism = true;
                this.finished = false;
                this.previousPosition = strafe.getSensorPosition();
                System.out.println("Starting auto alignment");
                System.out.println(distTargIn.getDouble(99));
                System.out.println("Inches away from center: " + inchesToMove);
                if (inchesToMove > 0) {
                    strafe.set(-STRAFE_SPEED);
                    System.out.println("moving left");
                } else if (inchesToMove < 0) {
                    strafe.set(STRAFE_SPEED);
                    System.out.println("Moving right");
                } else {
                    System.out.println("wut");
                }
            }
            this.movedInches = (strafe.getSensorPosition() - this.previousPosition) / ENCODER_COUNTS_TO_IN;
            if (this.inchesToMove - this.movedInches > -.1 && this.inchesToMove - this.movedInches < .1) {
                strafe.set(0);
                this.movedInches = 0;
                this.movingHatchMechanism = false;
                this.finished = true;
                System.out.println("GOT EM");
            } else if (this.movedInches > this.inchesToMove) {
                strafe.set(STRAFE_SPEED);
                System.out.println("moving to the right");
            } else if (this.movedInches < this.inchesToMove) {
                strafe.set(-STRAFE_SPEED);
                System.out.println("moving to the left");
            }
        }

        @Override
        protected boolean isFinished() {
            return this.finished;
        }
    }

}