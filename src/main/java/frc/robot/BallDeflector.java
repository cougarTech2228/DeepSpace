package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallDeflector {
    private Solenoid leftArm;
    private Solenoid rightArm;

    public BallDeflector() {
        leftArm = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_3);
        rightArm = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_4);
        SmartDashboard.putBoolean("Left Arm", false);
        SmartDashboard.putBoolean("Right Arm", false);
    }

    public void teleOpPeriodic() {
        if (SmartDashboard.getBoolean("Left Arm", false)) { 
            extendLeftArm();
        } else {
            retractLeftArm();
        }
        
        if(SmartDashboard.getBoolean("Right Arm", false)) {
            extendRightArm();
        } else {
            retractRightArm();
        }
    }

    public void extendLeftArm() {
        leftArm.set(true);
    }

    public void extendRightArm() {
        rightArm.set(true);
    }

    public void retractLeftArm() {
        leftArm.set(false);
    }

    public void retractRightArm() {
        rightArm.set(false);
    }
}