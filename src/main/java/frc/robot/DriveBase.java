package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class DriveBase {
	
	public Navx navx;
	public XboxIF controls;
	public Motor RightFront;
	public Motor RightBack;
	public Motor LeftFront;
	public Motor LeftBack;
	public double DriveSpeedPercentage = 1;
	public double StrafeSpeedPercentage = 1;
	public double TurnSpeedPercentage = 0.5;
	
	public DriveBase(XboxIF controls, Navx navx) {
		this.controls = controls;
		this.navx = navx;

		RightFront = new Motor(RobotMap.RIGHT_FRONT);
		RightBack = new Motor(RobotMap.RIGHT_BACK);
		LeftFront = new Motor(RobotMap.LEFT_FRONT);
		LeftBack = new Motor(RobotMap.LEFT_BACK);
		
		RightFront.Invert(true);
		RightBack.Invert(true);
		
		RightFront.SetBrakeMode(true);
		RightBack.SetBrakeMode(true);
		LeftFront.SetBrakeMode(true);
		LeftBack.SetBrakeMode(true);

	}
	private double ZeroLimit(double input) {
		if(Math.abs(input) < 0.1)
			return 0;
		return input;
	}
	public void TeleopInit() {
		RightFront.TeleopInit();
		LeftFront.TeleopInit();
		RightBack.TeleopInit();
		LeftBack.TeleopInit();
	}
	public void AutoInit() {
		RightFront.AutoInit();
		LeftFront.AutoInit();
	}
	private double Limit(double input) {
		if(input > 1)
			input = 1;
		if(input < -1)
			input = -1;
		return input;
	}
	
	public void TeleopMove() {
		double Forward = controls.Throttle();
		double Strafe = controls.Strafe();
		double Turn = controls.Turn();
		double RightF, LeftF, RightB, LeftB;
		
		Forward = ZeroLimit(Forward);
		Strafe = ZeroLimit(Strafe);
		Turn = ZeroLimit(Turn);

		Forward *= DriveSpeedPercentage;
		Strafe *= StrafeSpeedPercentage;
		Turn *= TurnSpeedPercentage;

		RightF = Limit(Forward + Strafe + Turn);
		LeftF = Limit(Forward - Strafe - Turn);
		RightB = Limit(Forward - Strafe + Turn);
		LeftB = Limit(Forward + Strafe - Turn);

		RightFront.SetSpeed(RightF);
		LeftFront.SetSpeed(LeftF);
		RightBack.SetSpeed(RightB);
		LeftBack.SetSpeed(LeftB);		
	}
	public void TestEncoders() {
		if(controls.X_BUTTON()) {
			LeftFront.Set(0.5);
			System.out.println(LeftFront.GetSensorPosition());
		} else {
			LeftFront.Stop();
			LeftFront.SetEncoderToZero();
		}
		if(controls.Y_BUTTON()) {
			RightFront.Set(0.5);
			System.out.println(RightFront.GetSensorPosition());
		} else {
			RightFront.Stop();
			RightFront.SetEncoderToZero();
		}
		if(controls.A_BUTTON()) {
			LeftBack.Set(0.5);
			System.out.println(LeftBack.GetSensorPosition());
		} else {
			LeftBack.Stop();
			LeftBack.SetEncoderToZero();
		}
		if(controls.B_BUTTON()) {
			RightBack.Set(0.5);
			System.out.println(RightBack.GetSensorPosition());
		} else {
			RightBack.Stop();
			RightBack.SetEncoderToZero();
		}
	}
}