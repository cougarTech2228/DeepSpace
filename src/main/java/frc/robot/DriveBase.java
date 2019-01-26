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
		
		rightFront.invert(true);
		rightBack.invert(true);
		
		rightFront.setBrakeMode(true);
		rightBack.setBrakeMode(true);
		leftFront.setBrakeMode(true);
		leftBack.setBrakeMode(true);

	}
	private double zeroLimit(double input) {
		if(Math.abs(input) < 0.1)
			return 0;
		return input;
	}
	public void teleopInit() {
		rightFront.teleopInit();
		leftFront.teleopInit();

		//init back motors for mecanum
		if(mode == DriveType.Mecanum) {
			rightBack.teleopInit();
			leftBack.teleopInit();
		}
	}
	public void autoInit() {
		rightFront.autoInit();
		leftFront.autoInit();
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
		double Turn = controls.Turn();
		double RightF, LeftF, RightB, LeftB;
		
		Forward = zeroLimit(Forward);
		Turn = zeroLimit(Turn);

		Forward *= driveSpeedPercentage;
		Turn *= turnSpeedPercentage;

		if(mode == DriveType.Tank) {

			RightF = Limit(Forward + Turn);
			LeftF = Limit(Forward - Turn);
			
			rightFront.setSpeed(RightF);
			leftFront.setSpeed(LeftF);	
		}
		else if(mode == DriveType.Mecanum) {

			double Strafe = controls.Strafe();
			Strafe = zeroLimit(Strafe); 
			Strafe *= strafeSpeedPercentage;

			RightF = Limit(Forward + Strafe + Turn);
			LeftF = Limit(Forward - Strafe - Turn);
			RightB = Limit(Forward - Strafe + Turn);
			LeftB = Limit(Forward + Strafe - Turn);
	
			System.out.println(RightF);
			rightFront.setSpeed(RightF);
			leftFront.setSpeed(LeftF);
			rightBack.setSpeed(RightB);
			leftBack.setSpeed(LeftB);		
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
				rightFront.setSpeed(speed);
				leftFront.setSpeed(-speed);
			}
			else if(mode == DriveType.Mecanum) {
				rightFront.setSpeed(speed);
				rightBack.setSpeed(speed);
				leftFront.setSpeed(-speed);
				leftBack.setSpeed(-speed);
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
				rightFront.stop();
				leftFront.stop();
			}
			else if(mode == DriveType.Mecanum) {
				rightFront.stop();
				rightBack.stop();
				leftFront.stop();
				leftBack.stop();
			}
		}
	}
	//test
	public void TestEncoders() {
		if(controls.X_BUTTON()) {
			leftFront.set(0.5);
			System.out.println(leftFront.getSensorPosition());
		} else {
			leftFront.stop();
			leftFront.setEncoderToZero();
		}
		if(controls.Y_BUTTON()) {
			rightFront.set(0.5);
			System.out.println(rightFront.getSensorPosition());
		} else {
			rightFront.stop();
			rightFront.setEncoderToZero();
		}
		if(controls.A_BUTTON()) {
			leftBack.set(0.5);
			System.out.println(leftBack.getSensorPosition());
		} else {
			leftBack.stop();
			leftBack.setEncoderToZero();
		}
		if(controls.B_BUTTON()) {
			rightBack.set(0.5);
			System.out.println(rightBack.getSensorPosition());
		} else {
			rightBack.stop();
			rightBack.setEncoderToZero();
		}
	}
}