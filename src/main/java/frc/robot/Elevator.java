
package frc.robot;



public class Elevator {
    private DriverIF controls;
    private Motor frontLift;
    private Motor backLift; 
    private Motor liftDrive;
    private double speedFront = 0;
    private double speedBack = 0;
    private double speedDrive = 0;
    private double encoderCountBack = 0;
    private double encoderCountFront = 0;
    private double enocderCountDrive = 0;
    
public Elevator(){

    frontLift = new Motor (RobotMap.ACTION_MOTOR_1);
    backLift = new Motor (RobotMap.ACTION_MOTOR_2);
    liftDrive = new Motor (RobotMap.ACTION_MOTOR_3);


    }

public void TestRaise(){
    if(controls.elevatorUp()){
        frontLift.moveToEncoder(encoderCountFront, speedFront);
        backLift.moveToEncoder(encoderCountBack, speedBack);
        liftDrive.moveToEncoder(enocderCountDrive, speedDrive);
        }
    }
}