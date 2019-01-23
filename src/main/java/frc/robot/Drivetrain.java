package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.DrivingController;
import frc.robot.util.Odometer;
import frc.robot.util.SubsystemCommand;
import frc.robot.util.SubsystemModule;

public class Drivetrain extends SubsystemModule {

	// Drivetrain motors
	private CANSparkMax lMotor0 = new CANSparkMax(0, MotorType.kBrushless);
	private CANSparkMax lMotor1 = new CANSparkMax(1, MotorType.kBrushless);
	//private CANSparkMax lMotor2 = new CANSparkMax(2, MotorType.kBrushless);
	private CANSparkMax rMotor0 = new CANSparkMax(3, MotorType.kBrushless);
	private CANSparkMax rMotor1 = new CANSparkMax(4, MotorType.kBrushless);
	//private CANSparkMax rMotor2 = new CANSparkMax(5, MotorType.kBrushless);

	// PID controllers
	private CANPIDController lPidController = lMotor0.getPIDController();
	private CANPIDController rPidController = rMotor0.getPIDController();

	// MAX encoders
	private CANEncoder lEncoder = lMotor0.getEncoder();
	private CANEncoder rEncoder = rMotor0.getEncoder();

	// NavX
	private AHRS navX = new AHRS(SPI.Port.kMXP);

	// Differential drivetrain
	private DifferentialDrive drive = new DifferentialDrive(lMotor0, rMotor0);

	// PID coefficients
	private double velocity = 0;
	
	private double kMinOutput = -1;
	private double kMaxOutput = 1;

	private double kP = 6.0e-6;
	private double kI = 1.0e-7;
	private double kD = 0;
	private double kIS = 0;

	private double lKFF = 1.78e-4;
	private double rKFF = 1.71e-4;

	// Spline values
	private double x1 = 0;
	private double x2 = 0;
	private double x3 = 0;
	private double x4 = 0;

	private double y1 = 0;
	private double y2 = 0;
	private double y3 = 0;
	private double y4 = 0;

	private double acceleration = 0;
	private double maxVelocity = 0;
	private double startVelocity = 0;
	private double endVelocity = 0;

	private boolean forwards = true;


	// Robot characteristics
	private double wheelSeparation = 2;

	// Gearbox encoders
	private Encoder leftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, true, EncodingType.k4X);
	private Encoder rightEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, true,
			EncodingType.k4X);


	// Drivetrain initialization
	public Drivetrain() {
		
		drive.setSafetyEnabled(false);

		// Configure follow mode
		lMotor1.follow(lMotor0);
		//lMotor2.follow(lMotor0);
		rMotor1.follow(rMotor0);
		//rMotor2.follow(rMotor0);

		// Setup up PID coefficients
		lPidController.setP(kP);
		lPidController.setI(kI);
		lPidController.setD(kD);
		lPidController.setIZone(kIS);
		lPidController.setFF(lKFF);
		lPidController.setOutputRange(kMinOutput, kMaxOutput);

		rPidController.setP(kP);
		rPidController.setI(kI);
		rPidController.setD(kD);
		rPidController.setIZone(kIS);
		rPidController.setFF(rKFF);
		rPidController.setOutputRange(kMinOutput, kMaxOutput);

		// SmartDashboard configuration
		SmartDashboard.putNumber("Robot Velocity", velocity);
		
		SmartDashboard.putNumber("P Gain", kP);
		SmartDashboard.putNumber("I Gain", kI);
		SmartDashboard.putNumber("D Gain", kD);
		SmartDashboard.putNumber("I Saturation", kIS);

		SmartDashboard.putNumber("Left Feed Forward", lKFF);
		SmartDashboard.putNumber("Right Feed Forward", rKFF);

		SmartDashboard.putNumber("Max Output", kMaxOutput);
		SmartDashboard.putNumber("Min Output", kMinOutput);

		// For splines
		SmartDashboard.putNumber("x1", x1);
		SmartDashboard.putNumber("x2", x2);
		SmartDashboard.putNumber("x3", x3);
		SmartDashboard.putNumber("x4", x4);

		SmartDashboard.putNumber("y1", y1);
		SmartDashboard.putNumber("y2", y2);
		SmartDashboard.putNumber("y3", y3);
		SmartDashboard.putNumber("y4", y4);

		SmartDashboard.putNumber("Acceleration", acceleration);
		SmartDashboard.putNumber("Max Velocity", maxVelocity);
		SmartDashboard.putNumber("Start Velocity", startVelocity);
		SmartDashboard.putNumber("End Velocity", endVelocity);

		SmartDashboard.putBoolean("Forwards?", forwards);
	}


	// Instantiate odometer and link in encoders and navX
	public Odometer odometer = new Odometer(0,0,0) {

		@Override
		public void updateEncodersAndHeading() {
			this.headingAngle = 450 - navX.getFusedHeading();
			if(this.headingAngle>360) {
				this.headingAngle-=360;
			}	

			this.leftPos=leftEncoder.getDistance();
			this.rightPos=rightEncoder.getDistance();

			this.currentAverageVelocity = (leftEncoder.getRate() + rightEncoder.getRate()) / 2;
	
		}

	};

	// Instantiate point controller for autonomous driving
	public DrivingController drivingcontroller = new DrivingController(0.0005) {

		// Use output from odometer and pass into autonomous driving controller
		@Override
		public void updateVariables(){
			this.currentX = odometer.getCurrentX();
			this.currentY = odometer.getCurrentY();
			this.currentAngle = odometer.getHeadingAngle();
			this.currentAverageVelocity = odometer.getCurrentAverageVelocity();
		}

		// Link autonomous driving controller to the drive train motor control
		@Override
		public void driveRobot(double power, double pivot) {
			closedLoopArcade(power, pivot);
		}
	};

	// Setup initial state of the drivetrain
	public void init() {
		leftEncoder.reset();
		rightEncoder.reset();

		leftEncoder.setDistancePerPulse(0);
		rightEncoder.setDistancePerPulse(0);

		lMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rMotor0.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	// Pull values from SmartDashboard
	public void configureCoefficients() {
		velocity = SmartDashboard.getNumber("Robot Velocity", 0);
		
		double p = SmartDashboard.getNumber("P Gain", 0);
		double i = SmartDashboard.getNumber("I Gain", 0);
		double d = SmartDashboard.getNumber("D Gain", 0);
		double iS = SmartDashboard.getNumber("I Saturation", 0);

		double lFF = SmartDashboard.getNumber("Left Feed Forward", 0);
		double rFF = SmartDashboard.getNumber("Right Feed Forward", 0);
						
		double max = SmartDashboard.getNumber("Max Output", 0);
		double min = SmartDashboard.getNumber("Min Output", 0);

		SmartDashboard.putNumber("Graph Left Velocity", getCurrentLeftVelocity());
		SmartDashboard.putNumber("Graph Right Velocity", getCurrentRightVelocity());

		SmartDashboard.putNumber("Left Velocity", getCurrentLeftVelocity());
		SmartDashboard.putNumber("Right Velocity", getCurrentRightVelocity());

		SmartDashboard.putNumber("Left PID Output", lMotor0.getAppliedOutput());
		SmartDashboard.putNumber("Right PID Output", rMotor0.getAppliedOutput());

		SmartDashboard.putString("Encoder Values", getEncoderValues());
		SmartDashboard.putNumber("NavX Heading: ", navX.getFusedHeading());

		// If PID coefficients on SmartDashboard have changed, write new values to controller
		if (p != kP) { lPidController.setP(p); rPidController.setP(p); kP = p; }						
		if (i != kI) { lPidController.setI(i); rPidController.setI(i); kI = i; }
		if (d != kD) { lPidController.setD(d); rPidController.setD(d); kD = d; }			
		if (iS != kIS) { lPidController.setIZone(iS); lPidController.setIZone(iS); kIS = iS; }
		
		if (lFF != lKFF) { lPidController.setFF(lFF); lKFF = lFF; }
		if (rFF != rKFF) { rPidController.setFF(rFF); rKFF = rFF; }
						
		if ((max != kMaxOutput) || (min != kMinOutput)) { 
			lPidController.setOutputRange(min, max);
			rPidController.setOutputRange(min, max); 
			kMinOutput = min; kMaxOutput = max;
		}
	}

	public void setSplineValues() {
		SmartDashboard.getNumber("x1", 0);
		SmartDashboard.getNumber("x2", 0);
		SmartDashboard.getNumber("x3", 0);
		SmartDashboard.getNumber("x4", 0);

		SmartDashboard.getNumber("y1", 0);
		SmartDashboard.getNumber("y2", 0);
		SmartDashboard.getNumber("y3", 0);
		SmartDashboard.getNumber("y4", 0);

		SmartDashboard.getNumber("Acceleration", 0);
		SmartDashboard.getNumber("Max Velocity", 0);
		SmartDashboard.getNumber("Start Velocity", 0);
		SmartDashboard.getNumber("End Velocity", 0);

		SmartDashboard.getBoolean("Forwards?", true);
	}

	// General arcade drive
	public void arcadeDrive(double power, double pivot) {
		drive.arcadeDrive(power, pivot);
	}

	// Closed loop velocity based tank
	public void closedLoopTank(double leftVelocity, double rightVelocity) {
		configureCoefficients();
		
		lPidController.setReference(leftVelocity, ControlType.kVelocity);
		rPidController.setReference(-rightVelocity, ControlType.kVelocity);
	}

	// Closed loop arcade based tank
	public void closedLoopArcade(double velocity, double rps) {
		double pivot = Math.PI * wheelSeparation * rps;
		closedLoopTank(velocity - pivot, velocity + pivot);
	}

	// Output encoder values
	public String getEncoderValues() {
		String str = "LE_Motor: " + lEncoder.getPosition() + " RE_Motor: " + rEncoder.getPosition();
		str += "\nLE_Shaft: " + leftEncoder.get() + " RE_Shaft: " + rightEncoder.get();
		
		return str;
	}

	public double getVelocity() {
		return velocity;
	}

	public double getCurrentLeftVelocity() {
		return lEncoder.getVelocity();
	}

	public double getCurrentRightVelocity() {
		return -rEncoder.getVelocity();
	}

	// Output PID values
	public void getPIDValues() {
		System.out.println("kP: " + kP + " kI: " + kI + " lFF: " + lKFF + " rFF: " + rKFF);
	}

	// Disable drivetrain
	public void destruct() {
		// lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
		// rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);

		lMotor0.set(0);
		rMotor0.set(0);
	}

		// Subsystem run function, use controller collection (multi-threaded at fast period)
		@Override
		public void run() {
	
			// Run every time
			this.odometer.integratePosition();
	
			// Run only when subsystem is enabled
			if (this.enabled) {
				this.drivingcontroller.run();
			}
		}

	@Override
	public void registerCommands() {
		new SubsystemCommand(this.registeredCommands, "test_spline") {

			@Override
			public void initialize() {
				setSplineValues();
				drivingcontroller.addSpline(x1, x2, x3, x4, y1, y2, y3, y4, 
						acceleration, maxVelocity, startVelocity,endVelocity, forwards);
				drivingcontroller.run();
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {
				
			}
		};

		new SubsystemCommand(this.registeredCommands, "start_path") {

			@Override
			public void initialize() {
				enabled = true;
			}

			@Override
			public void execute() {

			}

			@Override
			public boolean isFinished() {
				return true;
			}

			@Override
			public void end() {

			}
		};
	}
}