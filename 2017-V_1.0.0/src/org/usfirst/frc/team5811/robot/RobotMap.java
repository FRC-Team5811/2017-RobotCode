package org.usfirst.frc.team5811.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	// Motors
	public static Victor frontLeftDriveMotor;
	public static Victor frontRightDriveMotor;
	public static Victor backLeftDriveMotor;
	public static Victor backRightDriveMotor;
	
	public static RobotDrive driveTrain;
	
	public static Victor intakeMotor;

	public static Victor elevatorMotor;
	
	public static Encoder driveEncoder;
	
	public static DoubleSolenoid shifterCylinder;
	
	public static DoubleSolenoid reservoirCylinder;
	
	
	public static Encoder shooterEncoder;
	public static Spark rightShooterMotor;
	public static Spark leftShooterMotor;
	
	public static Victor leftClimberMotor;
	public static Victor rightClimberMotor;
	
	// NavX
	public static AHRS ahrs;
	
	
	//magic numbers of things/ports that are created elsewhere
	public static int joystickDriverSlot = 0;
	public static int joystickManipulatorSlot = 1;
	
	public static int CompressorChannel = 0;
	
	public static int PDPElevatorChannel = 3;
	public static int PDPIntakeChannel = 13;
	public static int PDPFrontRightMotorChannel = 15;
	public static int PDPBackRightMotorChannel = 14;
	public static int PDPFrontLeftMotorChannel = 0;
	public static int PDPBackLeftMotorChannel = 1;
	public static int PDPClimber1MotorChannel = 12;
	public static int PDPClimber2MotorChannel = 2;
	
	//create all the static references...
	public RobotMap(){
		// Motor port instantiating
		frontLeftDriveMotor = new Victor(9);
		frontRightDriveMotor = new Victor(4);
		backLeftDriveMotor = new Victor(8);
		backRightDriveMotor = new Victor(3);
		
		driveTrain = new RobotDrive(frontLeftDriveMotor, backLeftDriveMotor, frontRightDriveMotor, backRightDriveMotor);
		
		intakeMotor = new Victor(2);
		
		elevatorMotor = new Victor(6);
		
		driveEncoder = new Encoder(2,3,true,Encoder.EncodingType.k4X);
		driveEncoder.setMaxPeriod(1);
		driveEncoder.setDistancePerPulse(36);
		driveEncoder.setMinRate(10);
		driveEncoder.setSamplesToAverage(32);
		
		leftClimberMotor = new Victor(1);
		rightClimberMotor = new Victor(7);
		
		shifterCylinder = new DoubleSolenoid(2, 3);
		shifterCylinder.set(DoubleSolenoid.Value.kForward);
		
		reservoirCylinder = new DoubleSolenoid(6, 7);
		reservoirCylinder.set(DoubleSolenoid.Value.kForward);
		
		rightShooterMotor = new Spark(0);
		leftShooterMotor = new Spark(5);
		//motor is inverted control
		//leftShooterMotor.setInverted(true);
		
		// Encoder inits and instantiations
		shooterEncoder = new Encoder(0,1,true,Encoder.EncodingType.k4X);
		shooterEncoder.setMaxPeriod(1);
		shooterEncoder.setDistancePerPulse(36);
		shooterEncoder.setMinRate(10);
		shooterEncoder.setSamplesToAverage(32);
		
		// NavX instantiation
		try {
			ahrs = new AHRS(SerialPort.Port.kUSB);
			//ahrs = new AHRS(I2C.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("NavX instantiation error");
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
	}
}
