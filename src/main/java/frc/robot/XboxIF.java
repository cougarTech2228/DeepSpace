package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class XboxIF {
	private XboxController xbox;

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

	public boolean RB_BUTTON() {
		return xbox.getBumper(Hand.kRight);
	};

	public boolean LB_BUTTON() {
		return xbox.getBumper(Hand.kLeft);
	};

	public boolean LS_BUTTON() {
		return xbox.getStickButton(Hand.kLeft);
	};

	public boolean RS_BUTTON() {
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

	public boolean POV_UP() {
		if (xbox.getPOV(0) == 0) {
			return true;
		} else {
			return false;
		}
	}
	public boolean POV_RIGHT() {
		if (xbox.getPOV(0) == 90) {
			return true;
		} else {
			return false;
		}
	}
	public boolean POV_DOWN() {
		if (xbox.getPOV(0) == 180) {
			return true;
		} else {
			return false;
		}
		
	}
	public boolean POV_LEFT() {
		if (xbox.getPOV(0) == 270) {
			return true;
		} else {
			return false;
		}
	}
	public void RUMBLE(double rumbleSpeed) {
		xbox.setRumble(RumbleType.kLeftRumble, rumbleSpeed);
		xbox.setRumble(RumbleType.kRightRumble, rumbleSpeed);
	}
}
