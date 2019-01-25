package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Motor {
	public static ArrayList<Motor> AllMotors = new ArrayList<Motor>();
	private WPI_TalonSRX motor;
	public int timeout = 10;
	public boolean invertMotor = false;
	public boolean invertEncoder = false;
	public boolean closeLoopEnabled = true;
	public int stallAmpsMax = 100;
	public int stallAmpsAim = 10;
	public int maxStallTime = 6000;
	public int velocitySampleTime = 64;
	public int PID_IDx = 0;
	public int SLOT_IDx = 0;
	public int integralZone = 0;
	public int closedLoopError = 0;
	
	public double feedForwardGain = 1.5;
	public double proportionalGain = 2;
	public double integralGain = 0;
	public double derivativeGain = 10;
	
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
	public void Invert(boolean invert) {
		motor.setInverted(invert);
	}
	public Motor(int CanID, Motor Following) {
		motor = new WPI_TalonSRX(CanID);
		motor.setInverted(invertMotor);
		motor.clearStickyFaults(timeout);
		motor.set(ControlMode.Follower, Following.motor.getDeviceID());
	}
	public void AutoInit() {
		closeLoopEnabled = true;
		motor.selectProfileSlot(SLOT_IDx, PID_IDx);
		motor.configAllowableClosedloopError(PID_IDx, closedLoopError, timeout);
		motor.config_kF(PID_IDx, feedForwardGain, timeout);
		
		motor.config_kP(PID_IDx, proportionalGain, timeout);
		motor.config_kI(PID_IDx, integralGain, timeout); 
		motor.config_kD(PID_IDx, derivativeGain, timeout);
		motor.config_IntegralZone(PID_IDx, integralZone, timeout);
	}
	public void TeleopInit() {
		closeLoopEnabled = false;
		motor.selectProfileSlot(SLOT_IDx, PID_IDx);
		motor.configAllowableClosedloopError(PID_IDx, 0, timeout);
		motor.config_kF(PID_IDx, feedForwardGain, timeout);
		
		motor.config_kP(PID_IDx, 0, timeout);
		motor.config_kI(PID_IDx, 0, timeout); 
		motor.config_kD(PID_IDx, 0, timeout);
	}
	public double GetSensorPosition() {
		int polarity = invertEncoder ? -1 : 1;
		return polarity * motor.getSelectedSensorPosition(PID_IDx);
	}
	public double GetSensorVelocity() {
		return motor.getSelectedSensorVelocity(PID_IDx);
	}
	public double GetMotorCurrent() {
		return motor.getOutputCurrent();
	}
	public double GetClosedLoopErr() {
		return motor.getClosedLoopError(PID_IDx);
	}
	public double GetBusVoltage() {
		return motor.getBusVoltage();
	}
	public void SetEncoderPosition(int position) {
		motor.getSensorCollection().setQuadraturePosition(position, 25);
	}
	public void SetEncoderToZero() {
		motor.getSensorCollection().setQuadraturePosition(0, 25);
	}
	public void SetRamp(double milliseconds) {
		if(closeLoopEnabled)
			motor.configClosedloopRamp(milliseconds, timeout);
		else motor.configOpenloopRamp(milliseconds, timeout);
	}
	public void Set(double speed) {
		motor.set(speed);
	}
	public void Set(ControlMode mode, double value) {
		motor.set(mode, value);
	}
	public void SetSpeed(double speed) {
		motor.set(ControlMode.Velocity, 563.69 * speed);
	}
	public void Stop() {
		motor.stopMotor();
	}
	public void SetBrakeMode(boolean on) {
		motor.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
	}
	public boolean MagicMoveTo(double variable, double max, double speed) {
		double BaseSpeed = 0.2;
		double PercentComplete = Math.abs(variable) / max;
		
		if(PercentComplete < 0.3333333333333) {
			double PercentRamp = PercentComplete * 3;
			
			SetSpeed(BaseSpeed + PercentRamp * (1 - BaseSpeed));
		}
		else if(PercentComplete > 0.6666666666667) {
			double PercentRamp = (PercentComplete - 0.6666666666667) * 3;
			
			SetSpeed(BaseSpeed + PercentRamp * (1 - BaseSpeed));
		}
		else SetSpeed(speed);
		
		System.out.println("Percent Complete: " + PercentComplete);
		
		return (PercentComplete >= 1);
	}
	public boolean MoveTo(int encoderCount, double speed) {
		if(Math.abs(GetSensorPosition()) >= Math.abs(encoderCount)) {
			//Set(ControlMode.Velocity, SRXDriveBaseCfg.kCountsPerRevolution);
			Stop();
			return true;
		}
		Set(speed);
		return false;
	}
	public static void MoveMotors(int encoderCount, double speed, Motor... Motors) {
		boolean moving = true;
		while(moving) {
			for(Motor m : Motors) {
				System.out.println("Running!");
				boolean motorState = m.MoveTo(encoderCount, speed);
				if(motorState) {
					System.out.println("Done!" + m.GetSensorPosition());
					moving = false;
					break;
				}
			}
		}
		System.out.println("Stopping!");
		for(Motor m : Motors)
			m.Stop();
	}
}