package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

public class Hatches{
    private Solenoid sol1 = new Solenoid(RobotMap.PCM, RobotMap.SOL1);
    private Solenoid sol2 = new Solenoid(RobotMap.PCM, RobotMap.SOL2);
    private Compressor compress = new Compressor(RobotMap.COMPRESSOR);
    private XboxIF xbox;

    public boolean pressureSwitch = compress.getPressureSwitchValue();



    public Hatches(XboxIF _xbox){
        xbox =_xbox;
    }

    public boolean doPlace(){
        return xbox.RIGHT_BUMPER();
    }

    public void place(){
        if(doPlace()){
            sol1.set(true);
            sol2.set(true);

        }
        else{
            sol1.set(false);
            sol2.set(false);
        }
        
    }

}