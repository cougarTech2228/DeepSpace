package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

public class Hatch{
    private Solenoid left;
    private Solenoid right;
    private XboxIF controls;
    private Motor strafe;
    private Compressor compressor;


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
            right.set(true);
        }
    }

    public void hatchStrafe(){
        if(controls.DPAD_LEFT()){
            
        }
    }


}