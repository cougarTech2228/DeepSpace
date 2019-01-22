package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Motor {
	public static ArrayList<Motor> AllMotors = new ArrayList<Motor>();
	private WPI_TalonSRX Motor;
	public int Timeout = 10;
	public boolean InvertMotor = false;
	public boolean InvertEncoder = false;
	public boolean CloseLoopEnabled = true;
	public int StallAmpsMax = 100;
	public int StallAmpsAim = 10;
	public int MaxStallTime = 6000;
	public int VelocitySampleTime = 64;
	public int PID_IDx = 0;
	public int SLOT_IDx = 0;
	public int IntegralZone = 0;
	public int ClosedLoopError = 0;
	
	public double FeedForwardGain = 1.5;
	public double ProportionalGain = 2;
	public double IntegralGain = 0;
	public double DerivativeGain = 10;
    
	public static boolean MotorsFinished = false;
	
	/**
	 * Sets up a master motor
	 * @param CanID the CanID of the motor
	 */
	public Motor(int CanID) {
		Motor = new WPI_TalonSRX(CanID);
		
		
		Motor.configNominalOutputForward(0.0, Timeout);
		Motor.configNominalOutputReverse(0.0, Timeout);
		Motor.configPeakOutputForward(1, Timeout);
		Motor.configPeakOutputReverse(-1, Timeout);
		
		// Reverse TalonSRX if necessary
		Motor.setInverted(InvertMotor);
		
		// Configure voltage compensation mode and set max voltage to 11 volts
		Motor.configVoltageCompSaturation(11.0, Timeout);
		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		Motor.configVoltageMeasurementFilter(32, Timeout);
		Motor.enableVoltageCompensation(true);
		
		// set output zero (neutral) deadband at 4%
		Motor.configNeutralDeadband(0.04, Timeout);

		// Set up stall conditions in SRX for the drive train
		Motor.configPeakCurrentLimit(StallAmpsMax, Timeout);
		Motor.configPeakCurrentDuration(MaxStallTime, Timeout);
		Motor.configContinuousCurrentLimit(StallAmpsAim, Timeout);
		Motor.enableCurrentLimit(true);
		
		// Configure the velocity measurement period and sample window rolling average
		// Sample period in ms from supported sample periods-default 100ms period/64 sample window
		Motor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, Timeout);
		Motor.configVelocityMeasurementWindow(VelocitySampleTime, Timeout);
		
		// Set up encoder input
		Motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_IDx, Timeout);
		Motor.setSensorPhase(InvertEncoder);
		
		// Clear quadrature position
		Motor.clearStickyFaults(Timeout);
		Motor.getSensorCollection().setQuadraturePosition(0, Timeout);
		
		Motor.stopMotor();
		AllMotors.add(this);
	}
	/**
	 * Sets up a follower motor
	 * @param CanID     the CanID of the Motor
	 * @param Following the Motor to follow
	 */
	public void Invert(boolean invert) {
		Motor.setInverted(invert);
	}
	public Motor(int CanID, Motor Following) {
		Motor = new WPI_TalonSRX(CanID);
		Motor.setInverted(InvertMotor);
		Motor.clearStickyFaults(Timeout);
		Motor.set(ControlMode.Follower, Following.Motor.getDeviceID());
	}
	public void AutoInit() {
		CloseLoopEnabled = true;
		Motor.selectProfileSlot(SLOT_IDx, PID_IDx);
		Motor.configAllowableClosedloopError(PID_IDx, ClosedLoopError, Timeout);
		Motor.config_kF(PID_IDx, FeedForwardGain, Timeout);
		
		Motor.config_kP(PID_IDx, ProportionalGain, Timeout);
		Motor.config_kI(PID_IDx, IntegralGain, Timeout); 
		Motor.config_kD(PID_IDx, DerivativeGain, Timeout);
		Motor.config_IntegralZone(PID_IDx, IntegralZone, Timeout);
	}
	public void TeleopInit() {
		CloseLoopEnabled = false;
		Motor.selectProfileSlot(SLOT_IDx, PID_IDx);
		Motor.configAllowableClosedloopError(PID_IDx, 0, Timeout);
		Motor.config_kF(PID_IDx, FeedForwardGain, Timeout);
		
		Motor.config_kP(PID_IDx, 0, Timeout);
		Motor.config_kI(PID_IDx, 0, Timeout); 
		Motor.config_kD(PID_IDx, 0, Timeout);
	}
	public double GetSensorPosition() {
		int polarity = InvertEncoder ? -1 : 1;
		return polarity * Motor.getSelectedSensorPosition(PID_IDx);
	}
	public double GetSensorVelocity() {
		return Motor.getSelectedSensorVelocity(PID_IDx);
	}
	public double GetMotorCurrent() {
		return Motor.getOutputCurrent();
	}
	public double GetClosedLoopErr() {
		return Motor.getClosedLoopError(PID_IDx);
	}
	public double GetBusVoltage() {
		return Motor.getBusVoltage();
	}
	public void SetEncoderPosition(int position) {
		Motor.getSensorCollection().setQuadraturePosition(position, 25);
	}
	public void SetEncoderToZero() {
		Motor.getSensorCollection().setQuadraturePosition(0, 25);
	}
	public void SetRamp(double milliseconds) {
		if(CloseLoopEnabled)
			Motor.configClosedloopRamp(milliseconds, Timeout); 
		else Motor.configOpenloopRamp(milliseconds, Timeout);
	}
	public void Set(double speed) {
		Motor.set(speed);
	}
	public void Set(ControlMode mode, double value) {
		Motor.set(mode, value);
	}
	public void SetSpeed(double speed) {
		Motor.set(ControlMode.Velocity, 563.69 * speed);
	}
	public void Stop() {
		Motor.stopMotor();
	}
	public void SetBrakeMode(boolean on) {
		Motor.setNeutralMode(on ? NeutralMode.Brake : NeutralMode.Coast);
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