package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class DriverIF {
    private XboxIF xbox;
    private Toggler elevatorExtendToggle;
    private Toggler lightsToggle;
    private Toggler slowRobot;
    private double hatchPushTimer;
    private boolean childMode;

    public DriverIF() {
        xbox = new XboxIF(1);
        lightsToggle = new Toggler(2, true);
        elevatorExtendToggle = new Toggler(2, true);
        slowRobot = new Toggler(2, true);

        elevatorExtendToggle.state = 1;
    }

    public double throttle() {
        if (!childMode)
            return -xbox.LEFT_JOYSTICK_Y();
        else
            return -xbox.LEFT_JOYSTICK_Y() / 2;
    }

    public double strafe() {
        if (!childMode)
            return -xbox.LEFT_JOYSTICK_X();
        else
            return -xbox.LEFT_JOYSTICK_X() / 2;
    }

    public double turn() {
            return -xbox.RIGHT_JOYSTICK_X();
    }

    public boolean hatchExtend() {
        if (xbox.RIGHT_BUMPER()) {
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
        if (!childMode) {
            return xbox.A_BUTTON();
        } else
            return false;
    }

    public boolean autoRetrieve() {
        if (!childMode)
            return xbox.B_BUTTON();
        else
            return false;
    }

    public boolean btHome() {
        if (!childMode)
            return xbox.Y_BUTTON();
        else
            return false;
    }

    public boolean level2Climb() {
        if (!childMode)
            return xbox.LEFT_BUMPER();
        else
            return false;
    }

    public boolean manualClimb() {
        if (!childMode)
            return xbox.LEFT_BUMPER();
        else
            return false;
    }

    public boolean slowRuss() {
        return slowRobot.toggle(xbox.X_BUTTON()) == 1;
    }

    public boolean climb3ndLvl() {
        if (!childMode)
            return xbox.LEFT_BUMPER();
        else
            return false;
    }

    public boolean climb2ndLvl() {
        if (!childMode)
            return xbox.DPAD_DOWN();
        else
            return false;
    }

    public boolean leftBallDeflector() {
        if (!childMode)
            return xbox.DPAD_LEFT();
        else
            return false;
    }

    public boolean rightBallDeflector() {
        if (!childMode)
            return xbox.DPAD_RIGHT();
        else
            return false;
    }

    public void childMode(boolean child) {
        childMode = child;
    }
}