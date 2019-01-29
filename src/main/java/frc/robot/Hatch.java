package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

public class Hatch{
    private Solenoid left;
    private Solenoid right;
    private XboxIF controls;
    private Motor strafe;
    private Compressor compressor;
    private double strafeSpeed = .5;
    private boolean movingLeft, movingRight;


    public Hatch(XboxIF controls){
        left = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_0);
        right = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_1);
        strafe = new Motor(RobotMap.ORIENTATION_MOTOR_1);
        compressor = new Compressor(RobotMap.PCM);
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
        if(controls.DPAD_LEFT()){
            strafe.set(-strafeSpeed);
            System.out.println("Moving Left");
            movingRight = false;
            movingLeft = true;
        }
        else if(!controls.DPAD_LEFT() && movingLeft){
            movingLeft = false;
            strafe.set(0);
        }
        if(controls.DPAD_RIGHT()){
            strafe.set(strafeSpeed);
            System.out.println("Moving Right");
            movingRight = true;
            movingLeft = false;
        }
        else if (!controls.DPAD_RIGHT() && movingRight){
            movingRight = false;
            strafe.set(0);
        }
        System.out.println(strafe.getSensorPosition());
    }


}