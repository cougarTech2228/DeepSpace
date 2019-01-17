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
		RightFront = new Motor(4);
		RightBack = new Motor(3);
		LeftFront = new Motor(2);
		LeftBack = new Motor(1);
		
		LeftFront.Invert(true);
		LeftBack.Invert(true);
		
		RightFront.SetBrakeMode(true);
		RightBack.SetBrakeMode(true);
		LeftFront.SetBrakeMode(true);
		LeftBack.SetBrakeMode(true);
	}
	private double ZeroLimit(double input) {
		if(Math.abs(input) < 0.2)
			return 0;
		return input;
	}
	public void TeleopInit() {
		RightFront.TeleopInit();
		LeftFront.TeleopInit();
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
		Turn = ZeroLimit(Turn);

		Turn *= TurnSpeedPercentage;
		
		System.out.println(Controller.Throttle());
		
		RightF = Limit(Forward + Strafe - Turn);
		LeftF = Limit(Forward - Strafe + Turn);
		RightB = Limit(Forward - Strafe - Turn);
		LeftB = Limit(Forward + Strafe + Turn);


		
		//Right *= DriveSpeedPercentage;
		//Left  *= DriveSpeedPercentage;
		
		RightFront.Set(ControlMode.Velocity, RightF * 563.69);
		LeftFront.Set(ControlMode.Velocity, LeftF * 563.69);
		RightBack.Set(ControlMode.Velocity, RightB * 563.69);
		LeftBack.Set(ControlMode.Velocity, LeftB * 563.69);
	}
	public void SpinTo(double angle, double speed) {
		boolean RightMotorComplete = false, LeftMotorComplete = false;
		double NavAngle;
		do {
			//RightMotorComplete = RightFront.MagicMoveTo(NavAngle, angle, speed);
			//LeftMotorComplete  =  LeftFront.MagicMoveTo(NavAngle, angle, speed);
		} while (!RightMotorComplete || !LeftMotorComplete);
	}
	public void MoveTo(int encoders, double speed) {
		Motor.MoveMotors(encoders, speed, RightFront, LeftFront);
	}
	public void ResetEncoders() {
		RightFront.SetEncoderToZero();
		LeftFront.SetEncoderToZero();
	}
}