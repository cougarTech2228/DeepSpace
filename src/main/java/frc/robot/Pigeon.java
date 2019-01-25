package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

public class Pigeon{
    private PigeonIMU Pigeon;
    private double[] yawPitchRoll = new double[3];


    public Pigeon(int port){
        Pigeon = new PigeonIMU(port);

    }

    public void resetYaw(){
        Pigeon.setYaw(0);
    }

    public void calibrate(){
        Pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    }

    public void pigeonCheck(){
        System.out.println("Yaw: " +  getYaw() + "  Pitch: " + getPitch() + "  Roll: " + getRoll());
    }

    public double getYaw(){
        Pigeon.getYawPitchRoll(yawPitchRoll);
        return yawPitchRoll[0];
    }

    public double getPitch(){
        Pigeon.getYawPitchRoll(yawPitchRoll);
        return yawPitchRoll[1];
    }

    public double getRoll(){
        Pigeon.getYawPitchRoll(yawPitchRoll);
        return yawPitchRoll[2];
    }

}