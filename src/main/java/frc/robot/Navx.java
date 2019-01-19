package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;

public class Navx implements PIDOutput {
	private AHRS ahrs;
	static final double AngleSP = 0.0;
	static double rate = 0;
	static double kP = 0.0003;
	static final double kI = 0.00;
	static final double kD = 0.00;
	
	static final double kF = 0.00;
	static final double kToleranceDegrees = 2.0f;
    public enum port {
        USB, I2C
    }

	public Navx(Navx.port _port) {
		try {
				 if(_port == port.I2C) ahrs = new AHRS(I2C.Port.kOnboard);
			else if(_port == port.USB) ahrs = new AHRS(SerialPort.Port.kUSB);

		} catch (RuntimeException ex) {
			System.out.println("Error starting the navx");
		}
    }

	public void setZeroAngle(double gyro) {
		//ahrs.setAngleAdjustment(gyro);
	}

	public double getAngle() {
		SmartDashboard.putNumber("Navx angle", ahrs.getAngle());
		return ahrs.getAngle();

	}

	public double getYaw() {
		return ahrs.getYaw();
	}
	
	public double getAccel() {
		return ahrs.getRawAccelX();
	}
	
	public double getRoll() {
		return ahrs.getRoll();
	}
	public void zeroYaw() {
		ahrs.zeroYaw();
		System.out.println("ZEROED THE YAW!");
	}

	public double _PIDCorrection(double angle) {
		double error;
		error = getYaw() - angle;
		return kP * error;

	}
	public double getAngleCorrection() {
		double error;
		double Yaw = Math.floor(getYaw()*10000)/10000;
		error = AngleSP - Yaw;
		double correction = 1 *((kP * error) - (kD * rate));
		System.out.println("Yaw " + Yaw + "; correction " + correction);
		if (Math.abs(Yaw) < 0.07) {
			correction = 0.0;
		}
		return correction;
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
	}
	
	public double round(double d, int places) {
		double a = Math.pow(10, places);
		return Math.round(d * a) / a;
	}
}
