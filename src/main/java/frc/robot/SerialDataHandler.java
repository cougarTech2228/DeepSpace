package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

public class SerialDataHandler {
    public static final int SEARCHING_FOR_HEADER = 0;
    public static final int SEARCHING_FOR_DATA_1 = 1;
    public static final int SEARCHING_FOR_DATA_2 = 2;

    private int state = SEARCHING_FOR_HEADER;
    private int sensor1Data = 0;
    private int sensor2Data = 0;

    SerialPort port;

    public SerialDataHandler() {
        port = new SerialPort(9600, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
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
        // int bytes = port.getBytesReceived();
        port.setReadBufferSize(2);
        byte[] byteArray = port.read(2);

        if (byteArray.length == 2) {

            byte a = byteArray[0]; 
            byte b = byteArray[1];

            switch (state) {
            case SEARCHING_FOR_HEADER:
               
                if (b == 0xff) {
                    if (a == 0x01) {
                        state = SEARCHING_FOR_DATA_1;
                    } else if (a == 0x02) {
                        state = SEARCHING_FOR_DATA_2;
                    } else {
                        System.out.println("Invalid ID");
                    }
                }
                break;
                case SEARCHING_FOR_DATA_1:
                
                sensor1Data = convertData(a, b);
                state = SEARCHING_FOR_HEADER;

                break;
                case SEARCHING_FOR_DATA_2:
                
                sensor2Data = convertData(a, b);
                state = SEARCHING_FOR_HEADER;

                break;
                default:
                System.out.println("Unexcepted case");
                break;
            }

            int fin = (int) (Byte.toUnsignedInt(a));
            fin = fin | (int) (Byte.toUnsignedInt(b) << 8);

            //System.out.println(String.format("%02X ", fin));
        }
        else
        {
            System.out.println("Nothing on port");
        }
    }
}