
package frc.robot;



public class Elevator {
    private DriverIF controls;
    private Motor frontlift;
    private Motor backlift; 
    private Motor liftdrive;
    private double speed = 0;
    private int encoderCount = 0;
    
public Elevator(){

    frontlift = new Motor (RobotMap.ACTION_MOTOR_1);
    backlift = new Motor (RobotMap.ACTION_MOTOR_2);
    liftdrive = new Motor (RobotMap.ACTION_MOTOR_3);


    }

public void TeleopRaise(){
    if(controls.elevatorUp()){
        Motor.moveMotors(encoderCount, speed, backlift, frontlift);
        Motor.moveMotors(encoderCount, speed, liftdrive);
        }
    }
}