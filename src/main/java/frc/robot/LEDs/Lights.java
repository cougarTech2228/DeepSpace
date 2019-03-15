package frc.robot.LEDs;

import com.ctre.phoenix.CANifier;

import frc.robot.RobotMap;
import frc.robot.LEDUtilities.HsvToRgb;

public class Lights {
    private CANifier leds;
    private float R = 255;
    private float G = 0;
    private float B = 0;
    private int state = 0;
    private float speed = 10f;

    public Lights() {
        leds = new CANifier(RobotMap.CANIFIER);
    }

    public void loop() {
        if (state == 0) {
            B = 0;
            R -= speed;
            G += speed;
            if (G >= 255) {
                state = 1;
            }
        } else if (state == 1) {
            R = 0;
            G -= speed;
            B += speed;
            if (B >= 255) {
                state = 2;
            }
        } else if (state == 2) {
            G = 0;
            B -= speed;
            R += speed;
            if (R >= 255) {
                state = 0;
            }
        }
        //System.out.println("R: " + R + " G: " + G + " B: " + B);

        leds.setLEDOutput(limit(R), CANifier.LEDChannel.LEDChannelA);
        leds.setLEDOutput(limit(G), CANifier.LEDChannel.LEDChannelB);
        leds.setLEDOutput(limit(B), CANifier.LEDChannel.LEDChannelC);

    }
    private float limit(float RGB) {
        if(RGB < 0) {
            return 0;
        }
        else if(RGB > 255) {
            return 1;
        }
        return RGB / 255.0f;
    }
}