package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

public class Arduino {
    SerialPort port;

    public Arduino() {
        port = new SerialPort(9600, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
        port.reset();
    }

    public void test() {
        // int bytes = port.getBytesReceived();
        port.setReadBufferSize(2);
        byte[] byteArray = port.read(2);
        System.out.println("Arduino: ");
        byte a = byteArray[0], b = byteArray[1];
        int fin = (int)(Byte.toUnsignedInt(a));
        fin = fin | (int)(Byte.toUnsignedInt(b) << 8);
        System.out.println(fin);

    }
}