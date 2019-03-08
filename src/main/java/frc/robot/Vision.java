package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    private final String TABLE_KEY = "datatable";
    public final int DEFAULT_VALUE = 42069666;
    private Relay visionRelay;
    private NetworkTableInstance visionDataTableInst;
    private NetworkTable visionDataTable;
    // state, 0 for searching, 1 for acquiring, or 2 for locked
    private NetworkTableEntry targState;
    // distance to target in inches,
    private NetworkTableEntry distTargIn;
    // horizontal distance from center of target, positive being to the right
    private NetworkTableEntry horzOffToIn;
    private boolean inRange = false;
    public Vision() {
        visionDataTableInst = NetworkTableInstance.getDefault();
        visionDataTable = visionDataTableInst.getTable(TABLE_KEY);
        targState = visionDataTable.getEntry("targState");
        distTargIn = visionDataTable.getEntry("distTargetIn");
        horzOffToIn = visionDataTable.getEntry("horzOffToIn");
        visionRelay = new Relay(0, Relay.Direction.kForward);
        visionRelay.set(Relay.Value.kOn);
        
    }
    public void visionInit(){
        
    }
    public void teleop(){
        if(getDistanceFromTarget() < 48 && getDistanceFromTarget() > 18){
            if(getCameraState() == 2){
                inRange = true;
            }
            else{
                inRange = false;
            }
        }
        else{
            inRange = false;
        }
        SmartDashboard.putBoolean("In Range", inRange);
        SmartDashboard.putNumber("Target State", getCameraState());
    }
    public double getDistanceFromTarget() {
        return distTargIn.getDouble(DEFAULT_VALUE);
    }
    public double getStrafeFromTarget() {
        return horzOffToIn.getDouble(DEFAULT_VALUE);
    }
    public int getCameraState() {
        return (int)targState.getDouble(DEFAULT_VALUE);
    }
    public void setRelay(boolean state) {
        if(state) {
            visionRelay.set(Relay.Value.kOn);
        }
        else {
            visionRelay.set(Relay.Value.kOff);
        }
    }
}