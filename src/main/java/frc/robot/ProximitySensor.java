package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ProximitySensor{
    private Arduino arduino;
    private double mmToIn = 0.0393701;

    public ProximitySensor(Arduino ar, int id){
        arduino = ar;
    }


    public double distanceInches(){
        byte[] distance = arduino.readSerialPort();
        byte a = distance[0], b = distance[1];
        int inches = (int)(Byte.toUnsignedInt(a));
        inches = inches | (int)(Byte.toUnsignedInt(b) << 8);
        double ret = inches * mmToIn;
        SmartDashboard.putNumber("Proximity Inches", ret);
        return ret;
    }



}