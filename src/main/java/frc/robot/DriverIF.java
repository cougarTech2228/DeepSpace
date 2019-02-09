package frc.robot;

public class DriverIF {
    private XboxIF xbox;
    public DriverIF() {
        xbox = new XboxIF(1);
    }
    public double throttle(){
        return xbox.LEFT_JOYSTICK_Y();
    }
    public double strafe(){
        return xbox.LEFT_JOYSTICK_X();
    }
    public double turn(){
        return xbox.RIGHT_JOYSTICK_X();
    }
    public boolean hatchExtend(){
        return xbox.Y_BUTTON();
    }
    public boolean hatchRetract(){
        return xbox.B_BUTTON();
    }
    public boolean hatchStrafeLeft(){
        if(xbox.LEFT_TRIGGER() > 0){
            return true;
        }
        return false;
    }
    public boolean hatchStrafeRight(){
        if(xbox.RIGHT_TRIGGER() > 0){
            return true;
        }
        return false;
    }
    public boolean elevatorUp(){
        return xbox.START_BUTTON();
    }
    public boolean encoderTestLeftFront(){
        return xbox.X_BUTTON();
    }
    public boolean encoderTestLeftBack(){
        return xbox.A_BUTTON();
    }
    public boolean encoderTestRightFront(){
        return xbox.Y_BUTTON();
    }
    public boolean encoderTestRightBack(){
        return xbox.B_BUTTON();
    }
    public boolean autoAlign(){
        return xbox.A_BUTTON();
    }
    public boolean deployElevator(){
        return xbox.LEFT_BUMPER();
    }
}