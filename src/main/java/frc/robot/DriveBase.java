package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class DriveBase {
	
	public Navx Navx;
	public Motor RightFront;
	public Motor RightBack;
	public Motor LeftFront;
	public Motor LeftBack;
	public double DriveSpeedPercentage = 0.8;
	public double TurnSpeedPercentage = 0.5;
	
	public DriveBase() {
		RightFront = new Motor(1);
		RightBack = new Motor(2);
		LeftFront = new Motor(3);
		LeftBack = new Motor(4);
		
		LeftFront.Invert(true);
		LeftBack.Invert(true);
		
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
	
	public void TeleopMove(DriverIF Controller) {
		double Forward = Controller.Throttle();
		double Strafe = Controller.Strafe();
		double Turn = Controller.Turn();
		double RightF, LeftF, RightB, LeftB;
		
		Forward = ZeroLimit(Forward);
		Strafe = ZeroLimit(Strafe);
		Turn = ZeroLimit(Turn);

		Turn *= TurnSpeedPercentage;
		
		RightF = Limit(Forward + Strafe - Turn);
		LeftF = Limit(Forward - Strafe + Turn);
		RightB = Limit(Forward - Strafe - Turn);
		LeftB = Limit(Forward + Strafe + Turn);


		
		//Right *= DriveSpeedPercentage;
		//Left  *= DriveSpeedPercentage;
		if(Controller.xboxIF.X_BUTTON())
			LeftFront.Set(0.5);
			else LeftFront.Stop();
		if(Controller.xboxIF.Y_BUTTON())
			RightFront.Set(0.5);
			else RightFront.Stop();
		if(Controller.xboxIF.A_BUTTON())
			LeftBack.Set(0.5);
			else LeftBack.Stop();
		if(Controller.xboxIF.B_BUTTON())
			RightBack.Set(0.5);
			else RightBack.Stop();
		/*
		RightFront.Set(ControlMode.Velocity, RightF * 563.69);
		LeftFront.Set(ControlMode.Velocity, LeftF * 563.69);
		RightBack.Set(ControlMode.Velocity, RightB * 563.69);
		LeftBack.Set(ControlMode.Velocity, LeftB * 563.69);
		*/
		System.out.println("RF Speed: " + RightFront.GetSensorPosition());
		System.out.println("LF Speed: " + LeftFront.GetSensorPosition());
		System.out.println("RB Speed: " + RightBack.GetSensorPosition());
		System.out.println("LB Speed: " + LeftBack.GetSensorPosition());
	}
	public void SpinTo(double angle, double speed) {
		boolean RightMotorComplete = false, LeftMotorComplete = false;
		double NavAngle;
		do {
			//RightMotorComplete = RightFront.MagicMoveTo(NavAngle, angle, speed);
			//LeftMotorComplete  =  LeftFront.MagicMoveTo(NavAngle, angle, speed);
		} while (!RightMotorComplete || !LeftMotorComplete);
	}
	/*public void MoveTo(int encoders, double speed) {
		Motor.MoveMotors(encoders, speed, RightFront, LeftFront);
	}
	public void ResetEncoders() {
		//RightFront.SetEncoderToZero();
		//LeftFront.SetEncoderToZero();
	}*/
}