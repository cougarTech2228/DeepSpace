package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import jdk.jfr.Percentage;

public class DriveBase {
	
	private Navx navx;
	private XboxIF controls;
	private Motor rightFront;
	private Motor rightBack;
	private Motor leftFront;
	private Motor leftBack;
	private double driveSpeedPercentage = 1;
	private double strafeSpeedPercentage = 1;
	private double turnSpeedPercentage = 0.5;
	
	private DriveType mode;
	public enum DriveType {
		Mecanum, Tank
	}

	public DriveBase(XboxIF controls, Navx navx, DriveType mode) {
		this.controls = controls;
		this.navx = navx;
		this.mode = mode;

<<<<<<< HEAD
		rightFront = new Motor(1);
		leftFront = new Motor(3);
		
		//Mecanum
		if(mode == DriveType.Mecanum) {
			rightBack = new Motor(2);
			leftBack = new Motor(4);
		}
		//Tank
		else if(mode == DriveType.Tank) {
			rightBack = new Motor(2, rightFront);
			leftBack = new Motor(4, leftFront);
		}
=======
		RightFront = new Motor(RobotMap.RIGHT_FRONT);
		RightBack = new Motor(RobotMap.RIGHT_BACK);
		LeftFront = new Motor(RobotMap.LEFT_FRONT);
		LeftBack = new Motor(RobotMap.LEFT_BACK);
>>>>>>> b2323d08bbc470701eec918bcd6335510d78ced2
		
		rightFront.Invert(true);
		rightBack.Invert(true);
		
		rightFront.SetBrakeMode(true);
		rightBack.SetBrakeMode(true);
		leftFront.SetBrakeMode(true);
		leftBack.SetBrakeMode(true);

	}
	private double ZeroLimit(double input) {
		if(Math.abs(input) < 0.1)
			return 0;
		return input;
	}
	public void TeleopInit() {
		rightFront.TeleopInit();
		leftFront.TeleopInit();

		//init back motors for mecanum
		if(mode == DriveType.Mecanum) {
			rightBack.TeleopInit();
			leftBack.TeleopInit();
		}
	}
	public void AutoInit() {
		rightFront.AutoInit();
		leftFront.AutoInit();
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
<<<<<<< HEAD
=======
		double Strafe = controls.Strafe();
>>>>>>> b2323d08bbc470701eec918bcd6335510d78ced2
		double Turn = controls.Turn();
		double RightF, LeftF, RightB, LeftB;
		
		Forward = ZeroLimit(Forward);
		Turn = ZeroLimit(Turn);

		Forward *= driveSpeedPercentage;
		Turn *= turnSpeedPercentage;

		if(mode == DriveType.Tank) {

			RightF = Limit(Forward + Turn);
			LeftF = Limit(Forward - Turn);
			
			rightFront.SetSpeed(RightF);
			leftFront.SetSpeed(LeftF);	
		}
		else if(mode == DriveType.Mecanum) {

			double Strafe = controls.Strafe();
			Strafe = ZeroLimit(Strafe); 
			Strafe *= strafeSpeedPercentage;

			RightF = Limit(Forward + Strafe + Turn);
			LeftF = Limit(Forward - Strafe - Turn);
			RightB = Limit(Forward - Strafe + Turn);
			LeftB = Limit(Forward + Strafe - Turn);
	
			System.out.println(RightF);
			rightFront.SetSpeed(RightF);
			leftFront.SetSpeed(LeftF);
			rightBack.SetSpeed(RightB);
			leftBack.SetSpeed(LeftB);		
		}	
	}
	//auto
	public TurnToAngle TurnToAngle(Navx navx, double targetAngle, double speed) {
		return new TurnToAngle(navx, targetAngle, speed);
	}
	public class TurnToAngle extends Command {
		boolean running;
		double targetAngle;
		double percentComplete;
		double maxSpeed;
		double speed;
		Navx navx;
		
		public TurnToAngle(Navx navx, double targetAngle, double speed) {
			this.targetAngle = targetAngle;
			this.navx = navx;
			this.maxSpeed = speed;
			running = true;
			percentComplete = 0;
			this.speed = speed;
		}
		protected void initialize() {
			navx.zeroYaw();
		}
		public void execute() {
			double minimumSpeed = 0.15;

			//calculates percent of turn complete
			percentComplete = navx.getAngle() / targetAngle;
			double speedMultiplier = 1;

			//if there is 45 degs or less to go, do this:
			if(navx.getAngle() >= targetAngle - 45) {

				//percentRampComplete is the percent of 45 degs left to go
				double percentRampComplete = (targetAngle - navx.getAngle()) / 45;

				//sets speed multiplier to ramped down value,
				//with the lowest value possible being minimumSpeed
				speedMultiplier = percentRampComplete * (1 - minimumSpeed) + minimumSpeed;
			}
			//set speed to a ramped max speed or if not in the last 45 degs, set to max speed
			speed = maxSpeed * speedMultiplier;
			
			//move
			if(mode == DriveType.Tank) {
				rightFront.SetSpeed(speed);
				leftFront.SetSpeed(-speed);
			}
			else if(mode == DriveType.Mecanum) {
				rightFront.SetSpeed(speed);
				rightBack.SetSpeed(speed);
				leftFront.SetSpeed(-speed);
				leftBack.SetSpeed(-speed);
			}
		}
		@Override
		protected boolean isFinished() {
			if(Math.abs(1 - percentComplete) < 0.015) {
				return true;
			}
			else return false;
		}
		@Override
		protected void end() {
			if(mode == DriveType.Tank) {
				rightFront.Stop();
				leftFront.Stop();
			}
			else if(mode == DriveType.Mecanum) {
				rightFront.Stop();
				rightBack.Stop();
				leftFront.Stop();
				leftBack.Stop();
			}
		}
	}
	//test
	public void TestEncoders() {
		if(controls.X_BUTTON()) {
			leftFront.Set(0.5);
			System.out.println(leftFront.GetSensorPosition());
		} else {
			leftFront.Stop();
			leftFront.SetEncoderToZero();
		}
		if(controls.Y_BUTTON()) {
			rightFront.Set(0.5);
			System.out.println(rightFront.GetSensorPosition());
		} else {
			rightFront.Stop();
			rightFront.SetEncoderToZero();
		}
		if(controls.A_BUTTON()) {
			leftBack.Set(0.5);
			System.out.println(leftBack.GetSensorPosition());
		} else {
			leftBack.Stop();
			leftBack.SetEncoderToZero();
		}
		if(controls.B_BUTTON()) {
			rightBack.Set(0.5);
			System.out.println(rightBack.GetSensorPosition());
		} else {
			rightBack.Stop();
			rightBack.SetEncoderToZero();
		}
	}
}