package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Robot extends TimedRobot {

	public static final Drivetrain drivetrain = new Drivetrain();
	public static OI oi;

	@Override
	public void robotInit() {
		oi = new OI();
		drivetrain.drivetrainInit();
	}

	@Override
	public void disabledInit() {
		drivetrain.drivetrainDestruct();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		generalInit();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		generalInit();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		// Robot.drivetrain.closedLoopTank(500, 500);

/*
		// Arcade Drive
		if (Math.abs(oi.getLeftJoystick()) > .05 || Math.abs(oi.getRightJoystick()) > .05) {
			drivetrain.arcadeDrive(-oi.getLeftJoystick(), oi.getRightJoystick());
		} else {
			drivetrain.arcadeDrive(0, 0);
		}
*/
		// Output encoder stuff
		//drivetrain.getEncoderValues();
	}

	@Override
	public void testInit() {
		generalInit();
	}
	@Override
	public void testPeriodic() { }

	private void generalInit() {
		drivetrain.drivetrainInit();
	}
}