package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import java.util.ArrayList;

public class RevTOF {
  
	public static final double IMAGE_WIDTH = 320.0;
	public static final double GEAR_WIDTH_FT = 1.166;
	public static final int BLOCK_SIZE = 14;
	public static int TOF_ADDRESS = 0x52;
	private I2C port;
	private boolean inRange;
	private double offset;
	
	private ArrayList<TOFBlock> pixyBlocks = new ArrayList<>();
	
	public RevTOF() {
		
		port = new I2C(I2C.Port.kOnboard, TOF_ADDRESS );
		
	}
	public void read() {
		byte[] returned = new byte[5];
		device.read(0x51, 3, returned);
		int i = Byte.toUnsignedInt(returned[1]);
		// System.out.println(i);
	}


}