package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class Hatch{
    private Solenoid left;
    private Solenoid right;
    private DriverIF controls;
    private DriveBase dBase;
    private DigitalInput leftSwitch;
    private DigitalInput rightSwitch;
    private Motor strafe;
    private Compressor compressor;
    private double strafeSpeed = .5;
    private final int ENCODER_COUNTS_TO_IN = 54666;
    private final int ENCODER_COUNT_CENTER = 164000;

    private final String TABLE_KEY = "datatable";
    private NetworkTableInstance visionDataTableInst;
    private NetworkTable visionDataTable;
    //state, 0 for searching, 1 for acquiring, or 2 for locked
    private NetworkTableEntry targState;
    //distance to target in inches,
    private NetworkTableEntry distTargIn;
    //horizontal distance from center of target, positive being to the right
    private NetworkTableEntry horzOffToIn;
    private boolean movingHatchMechanism = false;
    private double previousPosition = 0;
    private double movedInches = 0;



    public Hatch(DriverIF controls, DriveBase dBase){
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
        distTargIn = visionDataTable.getEntry("distTargIn");
        horzOffToIn = visionDataTable.getEntry("horzOffToIn");

    }
    
    public void extend(){
        left.set(true);
        right.set(true);
    }

    public void retract(){
        left.set(false);
        right.set(false);
    }

    public void teleop(){
        compressor.setClosedLoopControl(true);
        if(controls.hatchExtend()){
            extend();
        }
        else if(controls.hatchRetract()){
            retract();
        }
        hatchStrafe();
    }
    public boolean moveHatchMechanismToIn(double inchesAwayFromCenter){
        if(!movingHatchMechanism){
            movingHatchMechanism = true;
            this.previousPosition = strafe.getSensorPosition();
            if(inchesAwayFromCenter > 0){
                strafe.set(-strafeSpeed);
            }
            else if(inchesAwayFromCenter < 0){
                strafe.set(strafeSpeed);
            }
        }
        this.movedInches = (strafe.getSensorPosition() - this.previousPosition) / ENCODER_COUNTS_TO_IN;
        if(inchesAwayFromCenter - movedInches > -.1 && inchesAwayFromCenter - movedInches < .1){
            strafe.set(0);
            this.movedInches = 0;
            movingHatchMechanism = false;
            return true;
        }
        else if(this.movedInches > inchesAwayFromCenter){
            strafe.set(strafeSpeed);
        }
        else if(this.movedInches < inchesAwayFromCenter){
            strafe.set(-strafeSpeed);
        }
        return false;
    }

    public void hatchStrafe(){
        if(!rightSwitch.get()){
            strafe.setEncoderPosition(0);;
        }
        if(controls.hatchStrafeLeft() && leftSwitch.get()){
            strafe.set(-strafeSpeed);
        }
        else if(controls.hatchStrafeRight() && rightSwitch.get()){
            strafe.set(strafeSpeed);
        }
        else {
            strafe.set(0);
        }
        System.out.println(strafe.getSensorPosition());
    }
    /**
     * Freakin' cool method to automatically align the hatch to the station with vision. Welcome to the future.
     */
    public void autoDeploy(){
        if(distTargIn.getDouble(-1) < 24){
            if(targState.getDouble(0) == 2.0){
                if(moveHatchMechanismToIn(horzOffToIn.getDouble(-4))){
                    extend();
                }
                else{
                    moveHatchMechanismToIn(horzOffToIn.getDouble(-4));
                }
            }
            else{
                System.out.println("Not locked on");
            }
        }
        else{
            System.out.println("Too far");
        }
    }


}