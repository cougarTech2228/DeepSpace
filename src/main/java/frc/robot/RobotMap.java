package frc.robot;

public class RobotMap {
    //Many of the mechanism motors are there just in case we want to use more than one motor for them, they probably won't do anything
    //Professor wanted those

    public static int PDP = 1;
    public static int PCM = 2;

    //Drive Motor ID's
    public static int RIGHT_FRONT = 11;
    public static int RIGHT_BACK = 12;
    public static int LEFT_FRONT = 13;
    public static int LEFT_BACK = 14;

    //Acquisition mechanisms
    public static int ACQUISITION_MOTOR_1 = 21;
    public static int ACQUISITION_MOTOR_2 = 22;
    public static int ACQUISITION_MOTOR_3 = 23;
    public static int ACQUISITION_MOTOR_4 = 24;
    public static int ACQUISITION_MOTOR_5 = 25;
    public static int ACQUISITION_MOTOR_6 = 26;
    public static int ACQUISITION_MOTOR_7 = 27;
    public static int ACQUISITION_MOTOR_8 = 28;
    public static int ACQUISITION_MOTOR_9 = 29;

    //Orientation mechanisms
    public static int ORIENTATION_MOTOR_1 = 31;
    public static int ORIENTATION_MOTOR_2 = 32;
    public static int ORIENTATION_MOTOR_3 = 33;
    public static int ORIENTATION_MOTOR_4 = 34;
    public static int ORIENTATION_MOTOR_5 = 35;
    public static int ORIENTATION_MOTOR_6 = 36;
    public static int ORIENTATION_MOTOR_7 = 37;
    public static int ORIENTATION_MOTOR_8 = 38;
    public static int ORIENTATION_MOTOR_9 = 39;

    //Action Motors
    public static int ACTION_MOTOR_1 = 41;
    public static int ACTION_MOTOR_2 = 42;
    public static int ACTION_MOTOR_3 = 43;
    public static int ACTION_MOTOR_4 = 44;
    public static int ACTION_MOTOR_5 = 45;
    public static int ACTION_MOTOR_6 = 46;
    public static int ACTION_MOTOR_7 = 47;
    public static int ACTION_MOTOR_8 = 48;
    public static int ACTION_MOTOR_9 = 49;

    //Display Devices
    public static int DISPLAY_DEVICE_1 = 51;
    public static int DISPLAY_DEVICE_2 = 52;
    public static int DISPLAY_DEVICE_3 = 53;
    public static int DISPLAY_DEVICE_4 = 54;
    public static int DISPLAY_DEVICE_5 = 55;
    public static int DISPLAY_DEVICE_6 = 56;
    public static int DISPLAY_DEVICE_7 = 57;
    public static int DISPLAY_DEVICE_8 = 58;
    public static int DISPLAY_DEVICE_9 = 59;

    //PCM ID's
    
	public final static int PCM_PORT_0 = 0;
	public final static int PCM_PORT_1 = 1;
	public final static int PCM_PORT_2 = 2;
	public final static int PCM_PORT_3 = 3;
	public final static int PCM_PORT_4 = 4;
	public final static int PCM_PORT_5 = 5;
	public final static int PCM_PORT_6 = 6;
    public final static int PCM_PORT_7 = 7;


    public final static int DIGITAL_INPUT_0 = 0;
    public final static int DIGITAL_INPUT_1 = 1;
    public final static int DIGITAL_INPUT_2 = 2;
    public final static int DIGITAL_INPUT_3 = 3;
    public final static int DIGITAL_INPUT_4 = 4;
    public final static int DIGITAL_INPUT_5 = 5;
    public final static int DIGITAL_INPUT_6 = 6;
    public final static int DIGITAL_INPUT_7 = 7;
    public final static int DIGITAL_INPUT_8 = 8;
    public final static int DIGITAL_INPUT_9 = 9;

    
    //Misc devices
    public static int PIGEONIMU = 61;
    public static int CANIFIER = 62;
    public static int LEFT_DISTANCE_SENSOR = 1;
    public static int RIGHT_DISTANCE_SENSOR = 2;

    //Constants between bots
    public static final int ENCODER_COUNTS_TO_IN_MULE = 54666;
    public static final int ENCODER_COUNT_CENTER_MULE = 164000;
    public static final int ENCODER_COUNT_TOTAL_RUSS = 1924;
    public static final int ENCODER_COUNTS_PER_IN_RUSS = 320;

}