package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class DriverIF {
	public XboxIF xboxIF;
	XboxIF xboxIF2;
	Toggler toggler;
	Toggler squeeze;
	public DriverIF() {
		xboxIF = new XboxIF(1);
		xboxIF2 = new XboxIF(2);
		toggler = new Toggler(2);
		squeeze = new Toggler(2);
	}

	public boolean fastSpeed(boolean toggle) {
		if(!toggle) {
		return xboxIF.Y_BUTTON();
		}
		else {
			Toggler toggler = new Toggler(2);
			if(toggler.toggle(xboxIF.Y_BUTTON()) == 1) {
			return true;
			}
		}
		return false;
	}
	public boolean lowerSpeed() {
		return xboxIF.X_BUTTON();
	}

	public boolean hookForward() {
		if (xboxIF.POV_UP() || xboxIF2.POV_UP()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean hookBackward() {
		if (xboxIF.POV_DOWN() || xboxIF2.POV_DOWN()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean cubeRotateToggle() {
		if (xboxIF.A_BUTTON() || xboxIF2.A_BUTTON()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean expulsion() {
		if (xboxIF.B_BUTTON() || xboxIF2.B_BUTTON()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean collectionToggle() {
		if (xboxIF.LB_BUTTON() || xboxIF2.LB_BUTTON()) {
			return true;
		} else {
			return false;
		}//
	}

	public double Turn() {
		return xboxIF.RIGHT_JOYSTICK_X();

	}

	public double Throttle() {
		return xboxIF.LEFT_JOYSTICK_Y();
	}

	public double Strafe() {
		return xboxIF.LEFT_JOYSTICK_X();
	}

	public boolean LowerElevator() {
		if (xboxIF.LEFT_TRIGGER() > 0.6 || xboxIF2.LEFT_TRIGGER() > 0.6)
			return true;
		else
			return false;
	}

	public boolean RaiseElevator() {
		if (xboxIF.RIGHT_TRIGGER() > 0.6 || xboxIF2.RIGHT_TRIGGER() > 0.6)
			return true;
		else
			return false;
	}

	public boolean squeezeToggle() {
		if (xboxIF.RB_BUTTON() || xboxIF2.RB_BUTTON()) {
			squeeze.toggle(true);
		}
		return squeeze.state == 1 ? true : false;
	}

	public boolean winchWindUp() {
		if (xboxIF.START_BUTTON() || xboxIF2.START_BUTTON()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean elevatorToggleUp() {
		if (xboxIF.POV_UP() || xboxIF2.POV_UP()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean elevatorToggleDown() {
		if (xboxIF.POV_DOWN() || xboxIF2.POV_DOWN()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean camSwitch() {
		return xboxIF.POV_LEFT();
	}
	public void rumbleSet(boolean on, double rumbleSpeed) {
		if(on) {
		xboxIF.RUMBLE(rumbleSpeed);
		}
		else {
			xboxIF.RUMBLE(0);
		}
	}

}
