
package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {
private boolean doClimb = false, deploy = false;

    private DriverIF controls;

    // private SerialDataHandler arduino = new SerialDataHandler();
    private int leftDistance = 0;
    private int rightDistance = 0;

    // The front of the bot is where the hatch mechanism is
    private Motor frontLift;
    private Motor backLift; 
    private Motor liftDrive;

    // Sensors
    private DigitalInput frontLiftRaised = new DigitalInput(RobotMap.DIGITAL_INPUT_2);
    private DigitalInput frontLiftLowered = new DigitalInput(RobotMap.DIGITAL_INPUT_2);
    private DigitalInput backLiftRaised = new DigitalInput(RobotMap.DIGITAL_INPUT_2);
    private DigitalInput backLiftLowered = new DigitalInput(RobotMap.DIGITAL_INPUT_2);

    private double frontLiftSpeedUp = 0.5;
    private double frontLiftSpeedDown = -0.4;
    private double backLiftSpeedUp = 0.5;
    private double backLiftSpeedDown = -0.4;
    private double liftDriveSpeed = 0;

    private int encoderCount = 0;

    private climb climbState = climb.PullRobotUp;
    private enum climb {
        PullRobotUp, MoveForward, LiftDriveMotorUp
    }
    
public Elevator(){
    frontLift = new Motor(RobotMap.ACTION_MOTOR_1);
    backLift = new Motor(RobotMap.ACTION_MOTOR_2);
    liftDrive = new Motor(RobotMap.ACQUISITION_MOTOR_3);
    }

public void TeleopRaise(){
    if(controls.elevatorUp()){
        doClimb = true;
    }

    if(doClimb){
        // leftDistance = arduino.getSensor1Position();
        // rightDistance = arduino.getSensor2Position();

        switch (climbState){
            case PullRobotUp:
                if(!frontLiftLowered.get()){
                    frontLift.set(frontLiftSpeedDown);
                }
                if(!backLiftLowered.get()){                    
                    backLift.set(-backLiftSpeedDown);
                }

                if(frontLiftLowered.get() && backLiftLowered.get()){
                    frontLift.set(0);
                    backLift.set(0);
                    climbState = climb.MoveForward;
                }
            break;

            case MoveForward:
                // if()
            break;

            case LiftDriveMotorUp:
                if(!frontLiftRaised.get()){
                    frontLift.set(frontLiftSpeedUp);
                }
            break;



        } 

    }

        }

    public void raiseElevator(){
        if(controls.deployElevator()){
            deploy = true;
        }

        if(deploy){
            if(!frontLiftRaised.get()){
                frontLift.set(frontLiftSpeedUp);
            }
            if(!backLiftRaised.get()){
                backLift.set(backLiftSpeedUp);
            }

            if(backLiftRaised.get() && frontLiftRaised.get()){
                frontLift.set(0);
                backLift.set(0);
                deploy = false;
            }
        }

    }
    
}