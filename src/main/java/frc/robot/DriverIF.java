package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class DriverIF {
    private XboxIF xbox;
    private double hatchTimer;
    private Toggler elevatorExtendToggle;
    private Toggler lightsToggle;

    public DriverIF() {
        xbox = new XboxIF(1);
        hatchTimer = 0;
        lightsToggle = new Toggler(2, true);
        elevatorExtendToggle = new Toggler(2, true);
        
        elevatorExtendToggle.state = 1;
    }

    public double throttle() {
        return xbox.LEFT_JOYSTICK_Y();
    }

    public double strafe() {
        return xbox.LEFT_JOYSTICK_X();
    }

    public double turn() {
        return xbox.RIGHT_JOYSTICK_X();
    }

    public boolean hatchExtend() {
        if(xbox.RIGHT_BUMPER()) {
            hatchTimer = Timer.getFPGATimestamp();
        }
        return hatchTimer + 1 >= Timer.getFPGATimestamp();
    }
    public boolean elevatorToggle() {
        elevatorExtendToggle.toggle(xbox.X_BUTTON());
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

    public boolean autoAlign() {
        return xbox.A_BUTTON();
    }

    public boolean level2Climb() {
        return xbox.LEFT_BUMPER();
    }

    public boolean manualClimb() {
        return xbox.RIGHT_BUMPER();
    }
}