package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Hatch{
    private Solenoid left;
    private Solenoid right;
    private XboxIF controls;


    public Hatch(XboxIF controls){
        left = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_0);
        right = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_1);
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
        if(controls.A_BUTTON()){
            extend();
        }
        else if(controls.B_BUTTON()){
            retract();
        }
    }


}