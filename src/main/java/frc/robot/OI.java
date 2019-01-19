package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

	// Controllers and button boxes
	Joystick xbox1 = new Joystick(RobotMap.p_xbox1);

	// Xbox buttons
	JoystickButton a = new JoystickButton(xbox1, 1);
	JoystickButton b = new JoystickButton(xbox1, 2);
	JoystickButton x = new JoystickButton(xbox1, 3);
	JoystickButton y = new JoystickButton(xbox1, 4);
	JoystickButton lb = new JoystickButton(xbox1, 5);
	JoystickButton rb = new JoystickButton(xbox1, 6);
	JoystickButton back = new JoystickButton(xbox1, 7);
	JoystickButton start = new JoystickButton(xbox1, 8);

	public double getLeftJoystick() {
		return xbox1.getRawAxis(1);
	}

	public double getRightJoystick() {
		return xbox1.getRawAxis(4);
	}

	public OI() {
		a.whileHeld(new setVelocity());
	}
}