package frc.robot;

import java.util.Arrays;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.SerialPort;
import jdk.jfr.Unsigned;

public class Arduino {
    SerialPort port;

    public Arduino() {
        port = new SerialPort(9600, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
        port.reset();
    }

    public byte[] readSerialPort() {
        // int bytes = port.getBytesReceived();
        port.setReadBufferSize(2);
        byte[] byteArray = port.read(2);

        if (byteArray.length == 2) {
            System.out.println("Arduino: ");
            byte a = byteArray[0], b = byteArray[1];
            int fin = (int) (Byte.toUnsignedInt(a));
            fin = fin | (int) (Byte.toUnsignedInt(b) << 8);
            System.out.println(fin);
        }
        else
        {
            System.out.println("Nothing on port");
        }
        
        return byteArray;
    }

    
}