package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import java.util.ArrayList;

public class RevTOF {
    I2C device;
	
	public RevTOF() {
		device = new I2C(I2C.Port.kOnboard, 0x52);
		
	}
	public void read() {
		byte[] returned = new byte[5];
		device.read(0x51, 3, returned);
		int i = Byte.toUnsignedInt(returned[1]);
		// System.out.println(i);
	}


}