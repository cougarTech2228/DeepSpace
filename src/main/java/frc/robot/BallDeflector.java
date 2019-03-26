package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallDeflector {
    private Solenoid leftArm;
    private Solenoid rightArm;
    private boolean isLeftArmExtended;
    private boolean isRightArmExtended;
    private DriverIF controls;

    public BallDeflector(DriverIF controls) {
        leftArm = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_3);
        rightArm = new Solenoid(RobotMap.PCM, RobotMap.PCM_PORT_4);
        SmartDashboard.putBoolean("Left Arm", false);
        SmartDashboard.putBoolean("Right Arm", false);
        isLeftArmExtended = false;
        isRightArmExtended = false;
        this.controls = controls;
    }

    public void teleOpPeriodic() {
        if (controls.leftBallDeflector()) { // check if dpad left is pressed
                extendLeftArm();
        } else {
                retractLeftArm();
        }

        if (controls.rightBallDeflector()) { // check if dpad right is pressed
                extendRightArm();
        } else {
                retractRightArm();
        }
    }
    
    public boolean getLeftArmExtended() {
        return isLeftArmExtended;
    }

    public boolean getRightArmExtended() {
        return isRightArmExtended;
    }

    public void extendLeftArm() {
        SmartDashboard.putBoolean("Left Arm", true);

        if (isLeftArmExtended == false) {
            leftArm.set(true);
            System.out.println("Extending left arm");
        }

        isLeftArmExtended = true;
    }

    public void extendRightArm() {
        SmartDashboard.putBoolean("Right Arm", true);
        
        if (isRightArmExtended == false) {
            rightArm.set(true);
            System.out.println("Extending right arm");
        }

        isRightArmExtended = true;
    }

    public void retractLeftArm() {
        SmartDashboard.putBoolean("Left Arm", false);
        
        if (isLeftArmExtended == true) {
            leftArm.set(false);
            System.out.println("Retracting left arm");
        }

        isLeftArmExtended = false;
    }

    public void retractRightArm() {
        SmartDashboard.putBoolean("Right Arm", false);
        
        if (isRightArmExtended == true) {
            rightArm.set(false);
            System.out.println("Retracting right arm");
        }

        isRightArmExtended = false;
    }
}