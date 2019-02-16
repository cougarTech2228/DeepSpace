package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase {

	private static double countsPerInch = 35.899;
	private Navx navx;
	private DriverIF controls;
	public Motor rightFront;
	private Motor rightBack;
	private Motor leftFront;
	private Motor leftBack;
	private double driveSpeedPercentage = 1;
	private double strafeSpeedPercentage = 1;
	private double turnSpeedPercentage = 0.5;
	private Pigeon pidgey;
	private boolean zeroPigeon;

	private DriveType mode;

	public enum DriveType {
		Mecanum, Tank
	}

	public DriveBase(DriverIF controls, Navx navx, Pigeon pidgey, DriveType mode) {
		this.controls = controls;
		this.navx = navx;
		this.mode = mode;
		this.pidgey = pidgey;
		zeroPigeon = false;

		rightFront = new Motor(RobotMap.RIGHT_FRONT);
		leftFront = new Motor(RobotMap.LEFT_FRONT);

		// Mecanum
		if (mode == DriveType.Mecanum) {
			rightBack = new Motor(RobotMap.RIGHT_BACK);
			leftBack = new Motor(RobotMap.LEFT_BACK);

			rightFront.invert(true);
			rightBack.invert(true);
		}
		// Tank
		else if (mode == DriveType.Tank) {
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
		if (Math.abs(input) < 0.1)
			return 0;
		return input;
	}

	public void teleopInit() {
		rightFront.teleopInit();
		leftFront.teleopInit();
		/*
		int maxRightVel = 272;
		int maxLeftVel = 253;

		double right = 111;
		double left = 106;

		rightFront.setMaxVelocity(272);
		leftFront.setMaxVelocity(253);
		
		//SmartDashboard.putNumber("right kP",  2.6);
		//SmartDashboard.putNumber("left kP", 2.4);
		//SmartDashboard.putBoolean("button", false);

		rightFront.motor.configMotionCruiseVelocity(maxRightVel / 2, 10);
		leftFront.motor.configMotionCruiseVelocity(maxLeftVel / 2, 10);

		rightFront.motor.configMotionAcceleration(maxRightVel / 2, 10);
		leftFront.motor.configMotionAcceleration(maxLeftVel / 2, 10);

		rightFront.motor.config_kF(0, 1023.0 / maxRightVel, 10);
		leftFront.motor.config_kF(0, 1023.0 / maxLeftVel, 10);

		rightFront.motor.config_kP(0, 0.5, 10);
		leftFront.motor.config_kP(0, 0.5, 10);

		rightFront.motor.config_kI(0, 0.001, 10);
		leftFront.motor.config_kI(0, 0.001, 10);

		rightFront.motor.config_IntegralZone(0, 20, 10);
		leftFront.motor.config_IntegralZone(0, 20, 10);

		rightFront.motor.config_kD(0, 5, 10);
		leftFront.motor.config_kD(0, 5, 10);

		// init back motors for mecanum
		if (mode == DriveType.Mecanum) {
			rightBack.teleopInit();
			leftBack.teleopInit();
		}*/
	}

	public void autoInit() {

		int maxRightVel = 272;
		int maxLeftVel = 253;

		double right = 111;
		double left = 106;

		rightFront.setMaxVelocity(maxRightVel);
		leftFront.setMaxVelocity(maxLeftVel);
		
		//SmartDashboard.putNumber("right kP",  2.6);
		//SmartDashboard.putNumber("left kP", 2.4);
		//SmartDashboard.putBoolean("button", false);

		rightFront.motor.configMotionCruiseVelocity(maxRightVel / 2, 10);
		leftFront.motor.configMotionCruiseVelocity(maxLeftVel / 2, 10);

		rightFront.motor.configMotionAcceleration(maxRightVel / 2, 10);
		leftFront.motor.configMotionAcceleration(maxLeftVel / 2, 10);

		

		double rightP = SmartDashboard.getNumber("right kP", 0);
		double leftP = SmartDashboard.getNumber("left kP", 0);
		double c = SmartDashboard.getNumber("left kI", 0);
		double d = SmartDashboard.getNumber("left kI", 0);
		double a = SmartDashboard.getNumber("right kD", 0);
		double b = SmartDashboard.getNumber("left kD", 0);
		double rightkF = SmartDashboard.getNumber("rightkF", 0);
		double leftkF = SmartDashboard.getNumber("leftkF", 0);
		int e = (int)SmartDashboard.getNumber("right Izone", 0);
		int f = (int)SmartDashboard.getNumber("left Izone", 0);

		rightFront.motor.config_kF(0, 1023.0 / maxRightVel, 10);
		leftFront.motor.config_kF(0, 1023.0 / maxLeftVel, 10);

		// rightFront.motor.config_kF(0, rightkF, 10);
		// leftFront.motor.config_kF(0, leftkF, 10);

		rightFront.motor.config_kP(0, rightP, 10);
		leftFront.motor.config_kP(0, leftP, 10);

		rightFront.motor.config_kI(0, c, 10);
		leftFront.motor.config_kI(0, d, 10);

		rightFront.motor.config_IntegralZone(0, e, 10);
		leftFront.motor.config_IntegralZone(0, f, 10);

		rightFront.motor.config_kD(0, a, 10);
		leftFront.motor.config_kD(0, b, 10);
		//SmartDashboard.putBoolean("button", false);

	}

	private double Limit(double input) {
		if (input > 1)
			input = 1;
		if (input < -1)
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

		if (mode == DriveType.Tank) {

			RightF = Limit(Forward + Turn);
			LeftF = Limit(Forward - Turn);

			if(Turn == 0) {
				/*
				//untested
				double rightE = rightFront.getSensorPosition();
				double leftE = leftFront.getSensorPosition();

				double avg = (rightE + leftE) / 2;

				double rightD = rightE - avg;
				double leftD = leftE - avg;

				double rightP = rightD / 50;
				double leftP = leftD / 50;

				RightF -= rightP;
				LeftF -= leftP;*/
			}

			//double angle = pidgey.getYaw();
			// System.out.println("RightF" + RightF);
			// System.out.println("LeftF" + LeftF);
			rightFront.set(RightF);
			leftFront.set(LeftF);
		} else if (mode == DriveType.Mecanum) {

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

	// auto
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

			// calculates percent of turn complete
			percentComplete = navx.getAngle() / targetAngle;
			double speedMultiplier = 1;

			// if there is 45 degs or less to go, do this:
			if (navx.getAngle() >= targetAngle - 45) {

				// percentRampComplete is the percent of 45 degs left to go
				double percentRampComplete = (targetAngle - navx.getAngle()) / 45;

				// sets speed multiplier to ramped down value,
				// with the lowest value possible being minimumSpeed
				speedMultiplier = percentRampComplete * (1 - minimumSpeed) + minimumSpeed;
			}
			// set speed to a ramped max speed or if not in the last 45 degs, set to max
			// speed
			speed = maxSpeed * speedMultiplier;

			// move
			if (mode == DriveType.Tank) {
				rightFront.setSpeed(speed);
				leftFront.setSpeed(-speed);
			} else if (mode == DriveType.Mecanum) {
				rightFront.setSpeed(speed);
				rightBack.setSpeed(speed);
				leftFront.setSpeed(-speed);
				leftBack.setSpeed(-speed);
			}
		}

		@Override
		protected boolean isFinished() {
			if (Math.abs(1 - percentComplete) < 0.015) {
				return true;
			} else
				return false;
		}

		@Override
		protected void end() {
			if (mode == DriveType.Tank) {
				rightFront.stop();
				leftFront.stop();
			} else if (mode == DriveType.Mecanum) {
				rightFront.stop();
				rightBack.stop();
				leftFront.stop();
				leftBack.stop();
			}
		}
	}

	public MoveToInches moveToInches(double targetEncoderInches, double speed) {
		return new MoveToInches(targetEncoderInches, speed);
	}

	public class MoveToInches extends CommandGroup {
		private double equationConstant;
		private double maxSpeed;
		private double targetEncoderCount;
		private double threshold = 430;
		private double initialSpeed = 0.25;
		private double endingSpeed = 0.15;
		private boolean leftRunning = true;
		private boolean rightRunning = true;

		public MoveToInches(double targetEncoderInches, double speed) {
			this.maxSpeed = speed;
			this.targetEncoderCount = targetEncoderInches * countsPerInch;
			
		}

		protected void initialize() {
			System.out.println("Setting encoders to zero");
			rightFront.setEncoderToZero();
			leftFront.setEncoderToZero();
			equationConstant = threshold * (initialSpeed - endingSpeed) - targetEncoderCount;
			pidgey.resetYaw();
		}
		public void execute() {
			double encoderRight = Math.abs(rightFront.getSensorPosition());
			double encoderLeft = Math.abs(leftFront.getSensorPosition());

			double percentComplete = (encoderLeft + encoderRight) / (2 * targetEncoderCount);

			double speedRight = calcSpeed(encoderRight);
			double speedLeft = calcSpeed(encoderLeft);

			double angle = pidgey.getYaw();

			System.out.println("angle: " + angle);

			speedRight *= (45 + angle) / 45.0;
			speedLeft *= (45 - angle) / 45.0;

			if(percentComplete > 0.95) {
				leftRunning = false;
				rightRunning = false;
				leftFront.stop();
				rightFront.stop();
			}
			else {
				leftFront.setSpeed(speedLeft);
				rightFront.setSpeed(speedRight);
			}

		}
		private double calcSpeed(double value) {
			//equation: y = -(|2x+a|+a)/2b + c: where x = speed, a = targetCounts, b = threshold * (initialSpeed - endingSpeed), c = 2*threshold, d = initialSpeed
			//to see how it works, graph it on desmos
			double speed = -(Math.abs(2 * value - targetEncoderCount + equationConstant) - targetEncoderCount + equationConstant) / (2 * threshold) + initialSpeed;
			System.out.println("Data: " + value + ", " + targetEncoderCount + ", " + threshold + ", " + speed);
			//make sure it starts at a low speed
			if(speed > Math.abs(maxSpeed)) {
				speed = Math.abs(maxSpeed);
			}
			if(maxSpeed < 0) {
				speed = -speed;
			}
			return speed;
		}
		@Override
		protected boolean isFinished() {
			return !leftRunning && !rightRunning;
		}
		@Override
		protected void end() {
			leftFront.stop();
			rightFront.stop();
		}
	}

	public double platformEncoderRight() {
		return rightFront.getSensorPosition();
	}

	public double platformEncoderLeft() {
		return leftFront.getSensorPosition();
	}

	public void stopMoving(){
		rightFront.set(0);
		leftFront.set(0);
	}

	public DriveToInch driveToInch(double targetInches, double speed) {
		return new DriveToInch(targetInches, speed);
	}

	public class DriveToInch extends CommandGroup {
		public DriveToInch(double targetInches, double speed) {
			if (mode == DriveType.Mecanum) {
				this.addParallel(rightFront.moveToEncoder(targetInches * countsPerInch, speed, rightBack));
				this.addParallel(leftFront.moveToEncoder(targetInches * countsPerInch, speed, leftBack));
			} else if (mode == DriveType.Tank) {
				//this.addParallel(leftFront.moveToEncoder(targetInches * countsPerInch, speed));
				//this.addParallel(rightFront.moveToEncoder(targetInches * countsPerInch, speed));
				this.addParallel(moveToInches(24, -0.5));
			}
		}
	}

	public void elevatorClimb(double speed, double target) {
		leftFront.moveToEncoder(target, speed);
		rightFront.moveToEncoder(target, speed);
	}

	// test
	public void TestEncoders() {
		//leftFront.set(1);
		//rightFront.set(1);
		double sped = SmartDashboard.getNumber("speed", 0);
		leftFront.setSpeed(sped);
		rightFront.setSpeed(sped);
		//leftFront.set(1);
		//rightFront.set(1);
		/*
		if (controls.encoderTestLeftFront()) {
			leftFront.set(0.5);
			System.out.println(leftFront.getSensorPosition());
		} else {
			leftFront.stop();
			leftFront.setEncoderToZero();
		}
		if (controls.encoderTestRightFront()) {
			rightFront.set(0.5);
			System.out.println(rightFront.getSensorPosition());
		} else {
			rightFront.stop();
			rightFront.setEncoderToZero();
		}
		if (controls.encoderTestLeftBack()) {
			leftBack.set(0.5);
			System.out.println(leftBack.getSensorPosition());
		} else {
			leftBack.stop();
			leftBack.setEncoderToZero();
		}
		if (controls.encoderTestRightBack()) {
			rightBack.set(0.5);
			System.out.println(rightBack.getSensorPosition());
		} else {
			rightBack.stop();
			rightBack.setEncoderToZero();
		}*/
	}
}