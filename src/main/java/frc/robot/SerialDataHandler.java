package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class SerialDataHandler {
    public static final int SEARCHING_FOR_HEADER = 0;
    public static final int SEARCHING_FOR_DATA_1 = 1;
    public static final int SEARCHING_FOR_DATA_2 = 2;


    private int state = SEARCHING_FOR_HEADER;
    private int sensor1Data = 0;
    private int sensor2Data = 0;

    SerialPort port;

    public SerialDataHandler(int baudRate, SerialPort.Port serialPort, int numBits, SerialPort.Parity serialParity, SerialPort.StopBits stopBits) {
        port = new SerialPort(baudRate, serialPort, numBits, serialParity, stopBits);
        port.setTimeout(0.03);
        port.reset();
    }

    public int getSensor1Data() {
        return sensor1Data;
    }

    public int getSensor2Data() {
        return sensor2Data;
    }

    private int convertData(byte a, byte b) {

        int sensorData = 0;

        sensorData = (int) (Byte.toUnsignedInt(a));
        sensorData = sensorData | (int) (Byte.toUnsignedInt(b) << 8);

        return sensorData;
    }

    public void readPort() {
        sensor1Data = -1;
        sensor2Data = -1;
        port.setReadBufferSize(2);
        byte[] byteArray = port.read(2);

        if (byteArray.length == 2) {

            byte upperbyte = byteArray[0]; //b
            byte lowerbyte = byteArray[1];

            ////System.out.printlnln(String.format("byteA: %02X ", a));
            ////System.out.printlnln(String.format("byteB: %02X ", b));

            switch (state) {
            case SEARCHING_FOR_HEADER:

                if ((int)(Byte.toUnsignedInt(upperbyte)) == 0xff) {
                    if ((int)(Byte.toUnsignedInt(lowerbyte)) == 0x01) {
                        state = SEARCHING_FOR_DATA_1;
                    } else if ((int)(Byte.toUnsignedInt(lowerbyte)) == 0x02) {
                        state = SEARCHING_FOR_DATA_2;
                    } else {
                        //System.out.printlnln("Invalid ID");
                    }
                }
                break;
            case SEARCHING_FOR_DATA_1:
                sensor1Data = convertData(lowerbyte, upperbyte);
                state = SEARCHING_FOR_HEADER;
                //System.out.printlnln(String.format("byteA: %02X ", lowerbyte));
            //System.out.printlnln(String.format("byteB: %02X ", upperbyte));

                break;
            case SEARCHING_FOR_DATA_2:
                sensor2Data = convertData(lowerbyte, upperbyte);
                state = SEARCHING_FOR_HEADER;
                //System.out.printlnln(Timer.getFPGATimestamp());
                break;
            default:
                //System.out.printlnln("Unexcepted case");
                break;
            }
        } else {
            //System.out.printlnln("Nothing on port");
        }
    }
}