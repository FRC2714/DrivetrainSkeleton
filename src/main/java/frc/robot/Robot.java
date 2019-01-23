package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.util.ControlsProcessor;

public class Robot extends TimedRobot {

	public static final Drivetrain drivetrain = new Drivetrain();
	public static Object util;

	public ControlsProcessor ControlsProcessor;

	@Override
	public void robotInit() {
			
		ControlsProcessor = new ControlsProcessor(500000, 1) {
			@Override
			public void registerOperatorControls() {
				append("test_spline", this.a);
			}
		};

		ControlsProcessor.registerController("DriveTrain", drivetrain);

		ControlsProcessor.start();
		drivetrain.init();
	}

	@Override
	public void disabledInit() {
		drivetrain.destruct();
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
		drivetrain.init();
	}
}