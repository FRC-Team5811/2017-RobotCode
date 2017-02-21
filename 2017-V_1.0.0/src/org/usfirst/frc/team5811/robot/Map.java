package org.usfirst.frc.team5811.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Map {
	
	public static int frontLeftDriveMotor = 9;
	public static int frontRightDriveMotor = 4;
	public static int backLeftDriveMotor = 8;
	public static int backRightDriveMotor = 3;
	
	public static int intakeMotor = 2;
	public static int rightShooterMotor = 0;
	public static int leftShooterMotor = 5;
	
	public static int leftClimberMotor = 1;
	public static int rightClimberMotor = 7;
	
	public static int elevatorMotor = 6;
	
	public static int shooterEncoderChannelA = 0;
	public static int shooterEncoderChannelB = 1;
	
	public static int driveEncoderChannelA = 2;
	public static int driveEncoderChannelB = 3;
	
	public static int joystickDriverSlot = 0;
	public static int joystickManipulatorSlot = 1;
	
	public static int shifterForwardChannel = 2;
	public static int shifterBackwardChannel = 3;
	
	public static int reservoirForwardChannel = 6;
	public static int reservoirBackwardChannel = 7;
	
	public static int CompressorChannel = 0;
	
}
