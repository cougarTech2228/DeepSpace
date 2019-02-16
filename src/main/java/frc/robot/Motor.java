package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import jdk.jfr.Threshold;

public class Motor {
	private static ArrayList<Motor> AllMotors = new ArrayList<Motor>();
	public WPI_TalonSRX motor;
	private int maxVelocity;
	private int timeout = 10;
	private boolean invertMotor = false;
	private boolean invertEncoder = false;
	private boolean closeLoopEnabled = true;
	private int stallAmpsMax = 100;
	private int stallAmpsAim = 10;
	private int maxStallTime = 6000;
	private int velocitySampleTime = 64;
	private int PID_IDx = 0;
	private int SLOT_IDx = 0;
	private int integralZone = 0;
	private int closedLoopError = 0;
	
	private double encoderOffset = 0;
	private double feedForwardGain = 0;
	private double proportionalGain = 0;
	private double integralGain = 0;
	private double derivativeGain = 10;
	
	/**
	 * Sets up a master motor
	 * @param CanID the CanID of the motor
	 */
	public Motor(int CanID) {
		motor = new WPI_TalonSRX(CanID);
		
		
		motor.configNominalOutputForward(0.0, timeout);
		motor.configNominalOutputReverse(0.0, timeout);
		motor.configPeakOutputForward(1, timeout);
		motor.configPeakOutputReverse(-1, timeout);
		
		// Reverse TalonSRX if necessary
		motor.setInverted(invertMotor);
		//motor.configMotionCruiseVelocity(sensorUnitsPer100ms)
		// Configure voltage compensation mode and set max voltage to 11 volts
		motor.configVoltageCompSaturation(11.0, timeout);
		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		motor.configVoltageMeasurementFilter(32, timeout);
		motor.enableVoltageCompensation(true);
		
		// set output zero (neutral) deadband at 4%
		motor.configNeutralDeadband(0.04, timeout);

		// Set up stall conditions in SRX for the drive train
		motor.configPeakCurrentLimit(stallAmpsMax, timeout);
		motor.configPeakCurrentDuration(maxStallTime, timeout);
		motor.configContinuousCurrentLimit(stallAmpsAim, timeout);
		motor.enableCurrentLimit(true);
		
		// Configure the velocity measurement period and sample window rolling average
		// Sample period in ms from supported sample periods-default 100ms period/64 sample window
		motor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, timeout);
		motor.configVelocityMeasurementWindow(velocitySampleTime, timeout);
		
		// Set up encoder input
		motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_IDx, timeout);
		motor.setSensorPhase(invertEncoder);
		
		// Clear quadrature position
		motor.clearStickyFaults(timeout);
		motor.getSensorCollection().setQuadraturePosition(0, timeout);
		
		motor.stopMotor();
		AllMotors.add(this);
	}
	/**
	 * Sets up a follower motor
	 * @param CanID     the CanID of the Motor
	 * @param Following the Motor to follow
	 */
	public void invert(boolean invert) {
		motor.setInverted(invert);
		motor.setSensorPhase(invert);
		invertEncoder = true;
	}
	public Motor(int CanID, Motor Following) {
		motor = new WPI_TalonSRX(CanID);
		motor.setInverted(invertMotor);
		motor.clearStickyFaults(timeout);
		motor.set(ControlMode.Follower, Following.motor.getDeviceID());
	}
	public void autoInit() {
		closeLoopEnabled = true;
		motor.selectProfileSlot(SLOT_IDx, PID_IDx);
		motor.configAllowableClosedloopError(PID_IDx, 0, timeout);
		motor.config_kF(PID_IDx, 1, timeout);
		
		motor.config_kP(PID_IDx, proportionalGain, timeout);
		motor.config_kI(PID_IDx, integralGain, timeout); 
		motor.config_kD(PID_IDx, derivativeGain, timeout);
		motor.config_IntegralZone(PID_IDx, integralZone, timeout);
	}
	public void teleopInit() {
		closeLoopEnabled = false;
		motor.selectProfileSlot(SLOT_IDx, PID_IDx);
		motor.configAllowableClosedloopError(PID_IDx, 0, timeout);
		motor.config_kF(PID_IDx, 0, timeout);
		
		motor.config_kP(PID_IDx, 0, timeout);
		motor.config_kI(PID_IDx, 0, timeout); 
		motor.config_kD(PID_IDx, 0, timeout);
	}
	public double getSensorPosition() {
		return motor.getSelectedSensorPosition(PID_IDx) - encoderOffset;
	}
	public double getSensorVelocity() {
		return motor.getSelectedSensorVelocity(PID_IDx);
	}
	public double getMotorCurrent() {
		return motor.getOutputCurrent();
	}
	public double getClosedLoopErr() {
		return motor.getClosedLoopError(PID_IDx);
	}
	public double getBusVoltage() {
		return motor.getBusVoltage();
	}
	public void setEncoderToZero() {
		encoderOffset = motor.getSelectedSensorPosition(PID_IDx);
	}
	public void resetEncoderPosition(int position) {
		motor.getSensorCollection().setQuadraturePosition(position, 25);
	}
	public void resetToZero() {
		motor.getSensorCollection().setQuadraturePosition(0, 25);
	}
	public void setMaxVelocity(int countsPer100ms) {
		maxVelocity = countsPer100ms;
	}
	public void setRamp(double milliseconds) {
		if(closeLoopEnabled)
			motor.configClosedloopRamp(milliseconds, timeout);
		else motor.configOpenloopRamp(milliseconds, timeout);
	}
	public void set(double speed) {
		motor.set(speed);
	}
	public void set(ControlMode mode, double value) {
		motor.set(mode, value);
	}
	public void setSpeed(double speed) {
		motor.set(ControlMode.Velocity, maxVelocity * speed);
	}
	public void stop() {
		motor.stopMotor();
	}
	public void setBrakeMode(boolean on) {
		motor.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
	}
	public MoveTo moveToEncoder(double targetEncoderCount, double speed, Motor...pairedMotors) {
		// System.out.println("Returning new moveTo");
		return new MoveTo(targetEncoderCount, speed, pairedMotors);
	}
	public class MoveTo extends Command {
		int state; //0=encoders, 1=navx, 2=pijy
		double[] slowThreshhold = new double[] {430, 45, 45};
		double initialSpeed = 0.25;
		double endingSpeed = 0.15;
		double equationConstant;
		boolean running;
		double target;
		double percentComplete;
		double maxSpeed;
		double speed;
		Navx navx;
		Pigeon pijy;
		Motor[] pairedMotors;

		public MoveTo(double targetEncoderCount, double speed, Motor...pairedMotors) {
			state = 0;
			this.pairedMotors = pairedMotors;
			this.maxSpeed = speed;
			this.target = targetEncoderCount;
			running = true;
			percentComplete = 0;
			this.speed = speed;
		}
		public MoveTo(double targetAngle, double speed, Navx navx, Motor...pairedMotors) {
			state = 1;
			this.navx = navx;
			this.pairedMotors = pairedMotors;
			this.maxSpeed = speed;
			this.target = targetAngle;
			running = true;
			percentComplete = 0;
			this.speed = speed;
		}
		public MoveTo(double targetAngle, double speed, Pigeon pijy, Motor...pairedMotors) {
			state = 2;
			this.pijy = pijy;
			this.pairedMotors = pairedMotors;
			this.maxSpeed = speed;
			this.target = targetAngle;
			running = true;
			percentComplete = 0;
			this.speed = speed;
		}
		public void setThreshold(double value) {
			slowThreshhold[state] = value;
		}
		protected void initialize() {
			System.out.println("Initializing MoveTo");
			running = true;
			percentComplete = 0;
			System.out.println("Setting encoders to zero");
			slowThreshhold[state] *= Math.abs(maxSpeed);
			equationConstant = slowThreshhold[state] * (initialSpeed - endingSpeed);
			setEncoderToZero();
		}
		public void execute() {
			double value = 0;
			//gets the value of the encoder/angle, if encoder, find the encoder counts of the paired motors
			if(state == 0) {
				value = Math.abs(getSensorPosition());
			}
			else if (state == 1) {
				value = Math.abs(navx.getAngle());
			}
			else if (state == 2) {
				value = Math.abs(pijy.getYaw());
			}
			//percent complete is self-explainitory
			percentComplete = Math.abs(value / target);
			
			//an equation Justin devised ramp up and down. It should ALWAYS output a positive number (target is target encoder count/angle, 
			//value is current encoder count/angle, slowThreshold is the amount of counts/degrees that the ramp up lasts)
			//Ex: if slowThreshold[0] == 430, then the robot will speed up for the first 430 counts until max speed is reached, then it will slow down for the last 430 counts
			double speed = calcSpeed(value);

			System.out.println("enc: " + getSensorPosition());
			System.out.println("Sped: " + speed);
			
			if(1 - percentComplete < 0.005) {
				System.out.println("Finished");
				running = false;
				speed = maxSpeed < 0 ? 0.2 : -0.2;
			}
			setSpeed(speed);
		}
		private double calcSpeed(double value) {
			double speed = -(Math.abs(2 * value - target + equationConstant) - target + equationConstant) / (2 * slowThreshhold[state]);
			System.out.println("Data: " + value + ", " + target + ", " + slowThreshhold[state]);
			//make sure it starts at a low speed
			speed += initialSpeed;
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
			
			return !running;
		}
		@Override
		protected void end() {
			System.out.println("Actually finished");
			stop();
			for(Motor m : pairedMotors) {
				m.stop();
			}
		}
	}
}