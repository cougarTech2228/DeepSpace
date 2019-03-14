package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class XboxIF {
	private XboxController xbox;

	//assignment declarations
	public double Throttle() { return LEFT_JOYSTICK_Y(); };
	public double Strafe() { return LEFT_JOYSTICK_X(); };
	public double Turn() { return RIGHT_JOYSTICK_X(); };

	public XboxIF(int _port) {
		xbox = new XboxController(_port);
	}

	public boolean A_BUTTON() {
		return xbox.getAButton();
	};

	public boolean B_BUTTON() {
		return xbox.getBButton();
	};

	public boolean X_BUTTON() {
		return xbox.getXButton();
	};

	public boolean Y_BUTTON() {
		return xbox.getYButton();
	};

	public boolean START_BUTTON() {
		return xbox.getStartButton();
	};

	public boolean BACK_BUTTON() {
		return xbox.getBackButton();
	};

	public boolean RIGHT_BUMPER() {
		return xbox.getBumper(Hand.kRight);
	};

	public boolean LEFT_BUMPER() {
		return xbox.getBumper(Hand.kLeft);
	};

	public boolean LEFT_JOYSTICK_PRESS() {
		return xbox.getStickButton(Hand.kLeft);
	};

	public boolean RIGHT_JOYSTICK_PRESS() {
		return xbox.getStickButton(Hand.kRight);
	};

	public double RIGHT_TRIGGER() {
		return xbox.getTriggerAxis(Hand.kRight);
	};

	public double LEFT_TRIGGER() {
		return xbox.getTriggerAxis(Hand.kLeft);
	};

	public double RIGHT_JOYSTICK_X() {
		return xbox.getX(Hand.kRight);
	};

	public double RIGHT_JOYSTICK_Y() {
		return xbox.getY(Hand.kRight);
	};

	public double LEFT_JOYSTICK_X() {
		return xbox.getX(Hand.kLeft);
	};

	public double LEFT_JOYSTICK_Y() {
		return xbox.getY(Hand.kLeft);
	};

	public boolean DPAD_UP() {
		int i = xbox.getPOV(0);
		return i == 0;
	}

	public boolean DPAD_RIGHT() {
		int i = xbox.getPOV(0);
		return i == 90;
	}

	public boolean DPAD_DOWN() {
		int i = xbox.getPOV(0);
		return i == 180;
	}

	public boolean DPAD_LEFT() {
		int i = xbox.getPOV(0);
		return i == 270;
	}

	public void RUMBLE(double rumbleSpeed) {
		xbox.setRumble(RumbleType.kLeftRumble, rumbleSpeed);
		xbox.setRumble(RumbleType.kRightRumble, rumbleSpeed);
	}

	public void RUMBLE_LEFT(double rumbleSpeed) {
		xbox.setRumble(RumbleType.kLeftRumble, rumbleSpeed);
	}

	public void RUMBLE_RIGHT(double rumbleSpeed) {
		xbox.setRumble(RumbleType.kRightRumble, rumbleSpeed);
	}

	public void RUMBLE_STOP() {
		RUMBLE(0);
	}
}
