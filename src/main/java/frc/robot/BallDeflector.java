package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class BallDeflector {
    private Solenoid leftArm;
    private Solenoid rightArm;

    public BallDeflector() {
        leftArm = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_3);
        rightArm = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_4);
    }
}