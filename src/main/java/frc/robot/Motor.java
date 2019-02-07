package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Command;

public class Motor {
	private static ArrayList<Motor> AllMotors = new ArrayList<Motor>();
	private WPI_TalonSRX motor;
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
	
	private double feedForwardGain = 1.5;
	private double proportionalGain = 2;
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
		motor.configAllowableClosedloopError(PID_IDx, closedLoopError, timeout);
		motor.config_kF(PID_IDx, feedForwardGain, timeout);
		
		motor.config_kP(PID_IDx, proportionalGain, timeout);
		motor.config_kI(PID_IDx, integralGain, timeout); 
		motor.config_kD(PID_IDx, derivativeGain, timeout);
		motor.config_IntegralZone(PID_IDx, integralZone, timeout);
	}
	public void teleopInit() {
		closeLoopEnabled = false;
		motor.selectProfileSlot(SLOT_IDx, PID_IDx);
		motor.configAllowableClosedloopError(PID_IDx, 0, timeout);
		motor.config_kF(PID_IDx, feedForwardGain, timeout);
		
		motor.config_kP(PID_IDx, 0, timeout);
		motor.config_kI(PID_IDx, 0, timeout); 
		motor.config_kD(PID_IDx, 0, timeout);
	}
	public double getSensorPosition() {
		int polarity = invertEncoder ? -1 : 1;
		return polarity * motor.getSelectedSensorPosition(PID_IDx);
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
	public void setEncoderPosition(int position) {
		motor.getSensorCollection().setQuadraturePosition(position, 25);
	}
	public void setEncoderToZero() {
		motor.getSensorCollection().setQuadraturePosition(0, 25);
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
		motor.set(ControlMode.Velocity, 563.69 * speed);
	}
	public void stop() {
		motor.stopMotor();
	}
	public void setBrakeMode(boolean on) {
		motor.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
	}
	public MoveTo moveToEncoder(double targetEncoderCount, double speed, Motor...pairedMotors) {
		System.out.println("Returning new moveTo");
		return new MoveTo(targetEncoderCount, speed, pairedMotors);
	}
	public class MoveTo extends Command {
		boolean running;
		double targetEncoderCount;
		double percentComplete;
		double maxSpeed;
		double speed;
		Motor[] pairedMotors;

		public MoveTo(double targetEncoderCount, double speed, Motor...pairedMotors) {

			this.pairedMotors = pairedMotors;
			this.maxSpeed = speed;
			this.targetEncoderCount = targetEncoderCount;
			running = true;
			percentComplete = 0;
			this.speed = speed;
		}
		protected void initialize() {
			System.out.println("Setting encoders to zero");
			setEncoderToZero();
		}
		public void execute() {
			double minimumSpeed = 0.15;

			//calculates percent of turn complete
			double encoder = Math.abs(getSensorPosition());
			percentComplete = Math.abs(encoder / targetEncoderCount);
			double speedMultiplier = 1;
			System.out.println("Encoders: " + getSensorPosition());

			//if there is 500 counts or less to go, do this:
			if(encoder >= targetEncoderCount - 500) {

				//percentRampComplete is the percent of the 500 counts left to go
				double percentRampComplete = (targetEncoderCount - encoder) / 500.0;

				//sets speed multiplier to ramped down value,
				//with the lowest value possible being minimumSpeed
				speedMultiplier = percentRampComplete * (1 - minimumSpeed) + minimumSpeed; 
			}
			//set speed to a ramped max speed or if not in the last 45 degs, set to max speed
			speed = maxSpeed * speedMultiplier;
			
			//move
			System.out.println("Moving: " + getSensorPosition() + ", "+ percentComplete + ", speed: " + speed);

			if(1 - percentComplete < 0.015) {
				System.out.println("Finished");
				running = false;
				speed = maxSpeed < 0 ? 0.2 : -0.2;
			}
			setSpeed(speed);
			for(Motor m : pairedMotors) {
				m.setSpeed(speed);
			}
		}
		@Override
		protected boolean isFinished() {
			return !running;
		}
		@Override
		protected void end() {
			stop();
			for(Motor m : pairedMotors) {
				m.stop();
			}
		}
	}
}