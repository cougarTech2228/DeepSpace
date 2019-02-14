package frc.robot.LEDUtilities;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap;

import com.ctre.phoenix.CANifier;

public class Hardware {
	public static CANifier canifier = new CANifier(RobotMap.CANIFIER);
	public static Joystick gamepad = new Joystick(0);
}