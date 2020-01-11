package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class DriverIF {
    private XboxIF xbox;
    private Toggler elevatorExtendToggle;
    private Toggler lightsToggle;
    private Toggler slowRobot;
    private double hatchPushTimer;

    public DriverIF() {
        xbox = new XboxIF(1);
        lightsToggle = new Toggler(2, true);
        elevatorExtendToggle = new Toggler(2, true);
        slowRobot = new Toggler(2, true);
        
        elevatorExtendToggle.state = 1;
    }

    public double throttle() {
        return -xbox.LEFT_JOYSTICK_Y();
    }

    public double strafe() {
        return -xbox.LEFT_JOYSTICK_X();
    }

    public double turn() {
        return -xbox.RIGHT_JOYSTICK_X();
    }

    public boolean hatchExtend() {
        if(xbox.RIGHT_BUMPER()) {
            hatchPushTimer = Timer.getFPGATimestamp();
        }
        return hatchPushTimer + 0.3 > Timer.getFPGATimestamp();
    }
    public boolean elevatorToggle() {
        elevatorExtendToggle.toggle(xbox.START_BUTTON());
        return elevatorExtendToggle.state == 1;
    }

    public boolean hatchStrafeLeft() {
        return xbox.LEFT_TRIGGER() > 0;
    }

    public boolean hatchStrafeRight() {
        return xbox.RIGHT_TRIGGER() > 0;
    }

    public boolean toggleLights() {
        lightsToggle.toggle(xbox.BACK_BUTTON());
        return lightsToggle.state == 0;
    }

    public boolean encoderTestLeftFront() {
        return xbox.X_BUTTON();
    }

    public boolean encoderTestLeftBack() {
        return xbox.A_BUTTON();
    }

    public boolean encoderTestRightFront() {
        return xbox.Y_BUTTON();
    }

    public boolean encoderTestRightBack() {
        return xbox.B_BUTTON();
    }

    public boolean relayTest() {
        return xbox.A_BUTTON();
    }

    public double encoderTestHatch() {
        return xbox.RIGHT_JOYSTICK_X();
    }

    public boolean autoDeploy() {
        return false;//return xbox.A_BUTTON();
    }
    public boolean manualOverride() {
        return xbox.A_BUTTON();
    }

    public boolean autoRetrieve(){
        return xbox.B_BUTTON();
    }
    public boolean btHome(){
        return xbox.Y_BUTTON();
    }

    public boolean level2Climb() {
        return xbox.LEFT_BUMPER();
    }

    public boolean manualClimb() {
        return xbox.LEFT_BUMPER();
    }
    public boolean slowRuss() {
        return slowRobot.toggle(xbox.X_BUTTON()) == 1;
    }
    public boolean climb3ndLvl() {
        return xbox.LEFT_BUMPER();
        
    }
    public boolean climb2ndLvl() {
        return xbox.DPAD_DOWN();
    }

    public boolean leftBallDeflector(){
        return xbox.DPAD_LEFT();
    }

    public boolean rightBallDeflector(){
        return xbox.DPAD_RIGHT();
    }
}