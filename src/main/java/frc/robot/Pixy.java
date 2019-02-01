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
	public void read() {
		byte[] returned = new byte[5];
		device.read(0x51, 5, returned);
		int i = Byte.toUnsignedInt(returned[1]);
		System.out.println(i);
	}
}