
package frc.robot;



public class Elevator {
    private DriverIF controls;
    private Motor front;
    private Motor back; 
    private double speed = 0;
    private int encoderCount = 0;
    
public Elevator(){


    front = new Motor (RobotMap.ACTION_MOTOR_1);
    back = new Motor (RobotMap.ACTION_MOTOR_2);



    }

public void TeleopRaise(){
    if(controls.elevatorUp()){
        Motor.MoveMotors(encoderCount, speed, front);
        Motor.MoveMotors(encoderCount, speed, back);
        }
    }
}