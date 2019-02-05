package frc.robot;

public class ProximitySensor{
    private Arduino arduino;
    private double mmToIn = 0.0393701;

    public ProximitySensor(Arduino ar){
        arduino = ar;
    }

    public double distanceInches(){
        double distance = arduino.test();
        distance = distance * mmToIn;
        return distance;
    }

}