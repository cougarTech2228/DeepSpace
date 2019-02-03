package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import jdk.jfr.Percentage;

public class DriveBase {
	
	private Navx navx;
	private DriverIF controls;
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

	public DriveBase(DriverIF controls, Navx navx, DriveType mode) {
		this.controls = controls;
		this.navx = navx;
		this.mode = mode;

		rightFront = new Motor(RobotMap.RIGHT_FRONT);
		leftFront = new Motor(RobotMap.LEFT_FRONT);
		
		//Mecanum
		if(mode == DriveType.Mecanum) {
			rightBack = new Motor(RobotMap.RIGHT_BACK);
			leftBack = new Motor(RobotMap.LEFT_BACK);
			
			rightFront.invert(true);
			rightBack.invert(true);
		}
		//Tank
		else if(mode == DriveType.Tank) {
			rightBack = new Motor(RobotMap.RIGHT_BACK, rightFront);
			leftBack = new Motor(RobotMap.LEFT_BACK, leftFront);

			leftFront.invert(true);
			leftBack.invert(true);
		}
		
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
		double Forward = controls.throttle();
		double Turn = controls.turn();
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

			double Strafe = controls.strafe();
			Strafe = zeroLimit(Strafe); 
			Strafe *= strafeSpeedPercentage;

			RightF = Limit(Forward + Strafe + Turn);
			LeftF = Limit(Forward - Strafe - Turn);
			RightB = Limit(Forward - Strafe + Turn);
			LeftB = Limit(Forward + Strafe - Turn);
			
			rightFront.setSpeed(RightF);
			leftFront.setSpeed(LeftF);
			rightBack.setSpeed(RightB);
			leftBack.setSpeed(LeftB);		
		}	
	}
	//auto
	public TurnToAngle TurnToAngle(double targetAngle, double speed) {
		return new TurnToAngle(targetAngle, speed);
	}
	public class TurnToAngle extends Command {
		boolean running;
		double targetAngle;
		double percentComplete;
		double maxSpeed;
		double speed;
		
		public TurnToAngle(double targetAngle, double speed) {
			this.targetAngle = targetAngle;
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

	public DriveToEncoder driveToEncoder(int targetEncoderCount, double speed) {
		return new DriveToEncoder(targetEncoderCount, speed);
	}
	public class DriveToEncoder extends Command {
		int targetEncoderCount;
		double speed;
		CommandGroup commands;
		public DriveToEncoder(int targetEncoderCount, double speed) {
			this.targetEncoderCount = targetEncoderCount;
			this.speed = speed;
			if(mode == DriveType.Mecanum) {
				commands.addParallel(rightFront.moveToEncoder(targetEncoderCount, speed, rightBack));
				commands.addParallel(leftFront.moveToEncoder(targetEncoderCount, speed, leftBack));
			} else if(mode == DriveType.Tank) {
				commands.addParallel(rightFront.moveToEncoder(targetEncoderCount, speed));
				commands.addParallel(leftFront.moveToEncoder(targetEncoderCount, speed));
			}
		}
		protected void initialize() {
			commands = new CommandGroup();
			commands.start();
		}
		public void execute() {
			
		}
		@Override
		protected boolean isFinished() {
			return commands.isCompleted();
		}
		@Override
		protected void end() {
			commands.close();
		}
	}
	//test
	public void TestEncoders() {
		if(controls.encoderTestLeftFront()) {
			leftFront.set(0.5);
			System.out.println(leftFront.getSensorPosition());
		} else {
			leftFront.stop();
			leftFront.setEncoderToZero();
		}
		if(controls.encoderTestRightFront()) {
			rightFront.set(0.5);
			System.out.println(rightFront.getSensorPosition());
		} else {
			rightFront.stop();
			rightFront.setEncoderToZero();
		}
		if(controls.encoderTestLeftBack()) {
			leftBack.set(0.5);
			System.out.println(leftBack.getSensorPosition());
		} else {
			leftBack.stop();
			leftBack.setEncoderToZero();
		}
		if(controls.encoderTestRightBack()) {
			rightBack.set(0.5);
			System.out.println(rightBack.getSensorPosition());
		} else {
			rightBack.stop();
			rightBack.setEncoderToZero();
		}
	}
}