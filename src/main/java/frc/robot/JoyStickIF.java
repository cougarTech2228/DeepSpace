package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class JoyStickIF{
    private Joystick joy;

    public JoyStickIF(int _port){
        joy = new Joystick(_port);
    }
    
    public double Axis_X(){
        return joy.getX();
    }
    //Returns how far to the sides it is pushed side/side (X)

    public double Axis_Y(){
        return joy.getY();
    }
    //Returns amount joystick is pushed foward/back (Y)

    public double Axis_Z(){
        return joy.getZ();
    }
    //Returns amount joystick is twisted left/right (Z)

    // public double Throttle(){
    //     return joy.getThrottle();    
    // }
    // //Returns amount joystick is pushed foward/back (Y)

    // public double Twist(){
    //     return joy.getTwist();
    // } 
    // //Returns amount joystick is twisted left/right (Z)


    public boolean Top_Button(){
        return joy.getTop();
    }
    //Returns whether top button is pressed

    public boolean Trigger(){
        return joy.getTrigger();
    }
    //Returns whether Trigger is being pulled

}