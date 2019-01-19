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

	private double kP = 0.00039;
	private double kI = 0;
	private double kD = 0;
	private double lKIS = 0;
	private double lKFF = 0;

	private double rKP = 0.00039;
	private double rKI = 0;
	private double rKIS = 0;
	private double rKD = 0;
	private double rKFF = 0;

	private int debugMode;

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
		lPidController.setP(lKP);
		lPidController.setI(lKI);
		lPidController.setD(lKD);
		lPidController.setIZone(lKIS);
		lPidController.setFF(lKFF);
		lPidController.setOutputRange(kMinOutput, kMaxOutput);

		rPidController.setP(rKP);
		rPidController.setI(rKI);
		rPidController.setD(rKD);
		rPidController.setIZone(rKIS);
		rPidController.setFF(rKFF);
		rPidController.setOutputRange(kMinOutput, kMaxOutput);

		// SmartDashboard configuration
		SmartDashboard.putNumber("Robot Velocity", velocity);
		
		SmartDashboard.putNumber("Left P Gain", lKP);
		SmartDashboard.putNumber("Left I Gain", lKI);
		SmartDashboard.putNumber("Left D Gain", lKD);
		SmartDashboard.putNumber("Left I Zone", lKIS);
		SmartDashboard.putNumber("Left Feed Forward", lKFF);

		SmartDashboard.putNumber("Right P Gain", rKP);
		SmartDashboard.putNumber("Right I Gain", rKI);
		SmartDashboard.putNumber("Right D Gain", rKD);
		SmartDashboard.putNumber("Right I Zone", rKIS);
		SmartDashboard.putNumber("Right Feed Forward", rKFF);

		SmartDashboard.putNumber("Max Output", kMaxOutput);
		SmartDashboard.putNumber("Min Output", kMinOutput);
		SmartDashboard.putNumber("Debug Mode", debugMode);
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
		
		double lP = SmartDashboard.getNumber("Left P Gain", 0);
		double lI = SmartDashboard.getNumber("Left I Gain", 0);
		double lD = SmartDashboard.getNumber("Left D Gain", 0);
		double lIS = SmartDashboard.getNumber("Left I Saturation", 0);
		double lFF = SmartDashboard.getNumber("Left Feed Forward", 0);

		double rP = SmartDashboard.getNumber("Right P Gain", 0);
		double rI = SmartDashboard.getNumber("Right I Gain", 0);
		double rD = SmartDashboard.getNumber("Right D Gain", 0);
		double rIS = SmartDashboard.getNumber("Right I Saturation", 0);
		double rFF = SmartDashboard.getNumber("Right Feed Forward", 0);
						
		double max = SmartDashboard.getNumber("Max Output", 0);
		double min = SmartDashboard.getNumber("Min Output", 0);

		SmartDashboard.putNumber("Current Left Velocity", getCurrentLeftVelocity());
		SmartDashboard.putNumber("Current Right Velocity", getCurrentRightVelocity());

		SmartDashboard.putNumber("Numerical Left Velocity", getCurrentLeftVelocity());
		SmartDashboard.putNumber("Numerical Right Velocity", getCurrentRightVelocity());

		SmartDashboard.putNumber("Left PID Output", lMotor0.getAppliedOutput());
		SmartDashboard.putNumber("Right PID Output", rMotor0.getAppliedOutput());

		
	
		// If PID coefficients on SmartDashboard have changed, write new values to controller
		if (lP != lKP) { lPidController.setP(lP); lKP = lP; }						
		if (lI != lKI) { lPidController.setI(lI); lKI = lI; }
		if (lD != lKD) { lPidController.setD(lD); lKD = lD; }			
		if (lIS != lKIS) { lPidController.setIZone(lIS); lKIS = lIS; }
		if (lFF != lKFF) { lPidController.setFF(lFF); lKFF = lFF; }

		if (rP != rKP) { rPidController.setP(rP); rKP = rP; }
		if (rI != rKI) { rPidController.setI(rI); rKI = rI; }
		if (rD != rKD) { rPidController.setD(rD); rKD = rD; }
		if (rIS != rKIS) { rPidController.setIZone(rIS); rKIS = rIS;}
		if (rFF != rKFF) { rPidController.setFF(rFF); rKFF = rFF; }
						
		if ((max != kMaxOutput) || (min != kMinOutput)) { 
			lPidController.setOutputRange(min, max); 
			kMinOutput = min; kMaxOutput = max;
		}
	}

	// General arcade drive
	public void arcadeDrive(double power, double pivot) {
		drive.arcadeDrive(power, pivot);
	}

	// Closed loop velocity based tank
	public void closedLoopTank(double leftVelocity, double rightVelocity) {
		
		if(SmartDashboard.getNumber("DebugMode", 0) > 0){
			configureCoefficients();																			
		}
				
		lPidController.setReference(leftVelocity, ControlType.kVelocity);
		rPidController.setReference(-rightVelocity, ControlType.kVelocity);
	}

	// Closed loop arcade based tank
	public void closedLoopArcade(double velocity, double rps) {
		double pivot = Math.PI * wheelSeparation * rps;
		closedLoopTank(velocity - pivot, velocity + pivot);
	}

	// Output encoder values
	public void getEncoderValues() {
		System.out.println("LE: " + lEncoder.getPosition() + " RE: " + rEncoder.getPosition());
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
		System.out.println("lKP: " + lKP + " lFF: " + lKFF);
		System.out.println("rKP: " + rKP + " rFF: " + rKFF);

	}

	// Disable drivetrain
	public void drivetrainDestruct() {
		// lMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);
		// rMotor0.setIdleMode(CANSparkMax.IdleMode.kBrake);

		lMotor0.set(0);
		rMotor0.set(0);
	}
}