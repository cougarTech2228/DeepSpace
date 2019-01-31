package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class Hatch{
    private Solenoid left;
    private Solenoid right;
    private XboxIF controls;
    private DigitalInput leftSwitch;
    private DigitalInput rightSwitch;
    private Motor strafe;
    private Compressor compressor;
    private double strafeSpeed = .5;


    public Hatch(XboxIF controls){
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
        if(controls.A_BUTTON()){
            // extend();
            left.set(true);
            System.out.println("Extend left");
        }
        else if(controls.X_BUTTON()){
            right.set(true);
            System.out.println("Extend right");
        }
        else if(controls.Y_BUTTON()){
            right.set(true);
            left.set(true);
        }
        else if(controls.B_BUTTON()){
            // retract();
            left.set(false);
            right.set(false);
        }
        hatchStrafe();
    }

    public void hatchStrafe(){
        if(!rightSwitch.get()){
            strafe.setEncoderPosition(0);;
        }
        if(controls.LEFT_TRIGGER() > 0 && leftSwitch.get()){
            strafe.set(-strafeSpeed);
            System.out.println("Moving Left");
        }
        else if(controls.RIGHT_TRIGGER() > 0 && rightSwitch.get()){
            strafe.set(strafeSpeed);
            System.out.println("Moving Right");
        }
        else {
            strafe.set(0);
        }
        System.out.println(strafe.getSensorPosition());
    }


}