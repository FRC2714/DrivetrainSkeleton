package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {

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
	}

	// Setup initial state of the drivetrain
	public void drivetrainInit() {
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
	public void drivetrainDestruct() {
		// lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
		// rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);

		lMotor0.set(0);
		rMotor0.set(0);
	}
}