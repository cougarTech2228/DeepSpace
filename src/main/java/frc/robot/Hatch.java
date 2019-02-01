package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class Hatch{
    private Solenoid left;
    private Solenoid right;
    private DriverIF controls;
    private DigitalInput leftSwitch;
    private DigitalInput rightSwitch;
    private Motor strafe;
    private Compressor compressor;
    private double strafeSpeed = .5;


    public Hatch(DriverIF controls){
        left = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_0);
        right = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_1);
        strafe = new Motor(RobotMap.ORIENTATION_MOTOR_1);
        strafe.setBrakeMode(true);
        compressor = new Compressor(RobotMap.PCM);
        leftSwitch = new DigitalInput(RobotMap.DIGITAL_INPUT_0);
        rightSwitch = new DigitalInput(RobotMap.DIGITAL_INPUT_1);
        this.controls = controls;
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


}