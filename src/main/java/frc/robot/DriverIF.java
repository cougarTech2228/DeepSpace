package frc.robot;

public class DriverIF {
    private XboxIF xbox;
    private Toggler elevatorExtendToggle;
    private Toggler lightsToggle;
    private Toggler slowRobot;

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
        return xbox.RIGHT_BUMPER();
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
        lightsToggle.toggle(xbox.START_BUTTON());
        return lightsToggle.state == 1;
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
}