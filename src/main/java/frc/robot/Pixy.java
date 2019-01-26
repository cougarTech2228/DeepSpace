package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class Pixy {
	I2C device;
	private byte[] requestPacket = {
		(byte)0xae, (byte)0xc1, (byte)0x30, (byte)0x02
	};

	public Pixy() {
		device = new I2C(I2C.Port.kOnboard, 0x54);
	}
	public int read() {
		byte[] returned = new byte[8];
		device.transaction(requestPacket, requestPacket.length, returned, returned.length);
		int i = 0;
		for(byte b : returned)
			i += (int)b;
		return i;
	}
}