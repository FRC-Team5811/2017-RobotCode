package org.usfirst.frc.team5811.robot;

import org.usfirst.frc.team5811.robot.commands.*;
import org.usfirst.frc.team5811.robot.subsystems.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public static Controls oi;
	
	Command autonomousCommand;
	SendableChooser chooser = new SendableChooser();

	// Controllers
	Joystick joyStickLeft;
	Joystick joyStickRight;

	// NavX
	AHRS ahrs;

	// Buttons
	JoystickButton logitechY;
	JoystickButton logitechA;
	JoystickButton logitechX;
	JoystickButton logitechB;
	JoystickButton logitechLeftBumper;
	JoystickButton logitechRightBumper;
	JoystickButton logitechLeftStickPress;
	JoystickButton logitechRightStickPress;
	JoystickButton logitechStart;
	JoystickButton logitechBack;
	JoystickButton logitechRightTrigger;
	JoystickButton logitechLeftTrigger;
	
	JoystickButton logitechY2;
	JoystickButton logitechA2;
	JoystickButton logitechX2;
	JoystickButton logitechB2;
	JoystickButton logitechLeftBumper2;
	JoystickButton logitechRightBumper2;
	JoystickButton logitechLeftStickPress2;
	JoystickButton logitechRightStickPress2;
	JoystickButton logitechStart2;
	JoystickButton logitechBack2;
	JoystickButton logitechRightTrigger2;
	JoystickButton logitechLeftTrigger2;

	// Motors
	Victor frontLeftDriveMotor;
	Victor frontRightDriveMotor;
	Victor backLeftDriveMotor;
	Victor backRightDriveMotor;
	
	Victor intake;
	
	Victor climberLeft;
	Victor climberRight;
	Victor elevator;

	// Encoder definitions and variables
	Encoder drive;
	
	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
	
	int rotationCount;
	double rotationRate;
	double shooterSpeedCorrection;
	boolean encDirection;
	boolean encIfStopped;
	double rotationPeriod;
	boolean spinUpComplete;
	boolean driveState;
		
	int rotationCountForDrive;
	double rotationRateForDrive;
	double distance;
	
	double autoSelecter;
	
	//Autonomous
	int cycleCounter;
	
	RobotStates autoMode; 
	
	String botPosition;
	String allianceColor;
	String chooseBoilerOrLoading;
	String baselineCross;
	String gearPlacement;
	String shoot;
	String shootAfterHopper;
	String hopperPickup;
	
	// Boolean state changes
	boolean shouldBeRunningSwitch;
	boolean wasPressedLeftStick;
	boolean shouldBeRunningShooter;
	boolean wasPressedRightBumper;
	boolean shouldBeRunningIntake;
	boolean wasPressedLeftBumper;
	boolean shouldBeRunningClimberUp;
	boolean wasPressedLogitechA;
	boolean shouldBeRunningClimberDown;
	boolean wasPressedLogitechY;
	boolean shouldBeRunningElevator;
	boolean wasPressedLogitechX;
	boolean shouldBeRunningGearTray;
	boolean wasPressedLogitechB;
	
	boolean shouldBeRunningShifter;
	boolean wasPressedRightStick;

	boolean shouldBeRunningCorrect;
	boolean wasPressedBackButton;
	
    boolean shouldBeRunningAutoTurn;
    boolean wasPressedStart;

	// Global Speed Values
	double leftSpeed;
	double rightSpeed;

	// A cylinder
	DoubleSolenoid shifterCylinder;
	DoubleSolenoid reservoirCylinder;

	
	CompressorSubsystem compressor = new CompressorSubsystem();
	PowerManagement power = new PowerManagement();

	float rotationPos;
	float macroPos;
	
	private void dualStick(){
		arcadeDrive(-joyStickLeft.getRawAxis(1),joyStickLeft.getRawAxis(2));
	}
	private void slowMove(double reduction){
		shifterCylinder.set(DoubleSolenoid.Value.kForward);
		arcadeDrive((-joyStickLeft.getRawAxis(1)*reduction), (joyStickLeft.getRawAxis(2)*reduction));
	}
	
	public void intakeOnOff(double speed){
		intake.set(speed);
	}
	
	/*
	private void switchDriveModes(){
		// SWITCHING BETWEEN DRIVE MODES
		if (logitechX.get()) {
			if (!wasPressedLeftStick) {
				shouldBeRunningSwitch = !shouldBeRunningSwitch;
			}
			wasPressedLeftStick = true;
			System.out.println("yah boi be on");
		} else {
			wasPressedLeftStick = false;
		}

		if (shouldBeRunningSwitch) {
			slowMove(0.5);
			joyStickLeft.setRumble(RumbleType.kLeftRumble, 1);
			joyStickLeft.setRumble(RumbleType.kRightRumble, 1);
		} else {
			dualStick();
			joyStickLeft.setRumble(RumbleType.kLeftRumble, 1);
			joyStickLeft.setRumble(RumbleType.kRightRumble, 0);
		}
	}
	*/
	
	private void testForCorrectionMode() {
		// CORRECTION MODE
		if (logitechBack.get()) {
			if (!wasPressedBackButton) {
				rotationPos = (float) ahrs.getAngle();
				shouldBeRunningCorrect = !shouldBeRunningCorrect;
			}
			wasPressedBackButton = true;
		} else {
			wasPressedBackButton = false;
		}

		if (shouldBeRunningCorrect) {
			correct();
			System.out.println("in correct mode");
		} else {
			System.out.println("not in correct mode");
		}
	}
	
	
	/*
	private void toggleIntake() {
		
		// INTAKE MY DUDES

		if (logitechLeftBumper2.get()) {
			if (!wasPressedLeftBumper) {
				shouldBeRunningIntake = !shouldBeRunningIntake;
			}
			wasPressedLeftBumper = true;
			// System.out.println("yah boi be on");
		} else {
			wasPressedLeftBumper = false;
		}

		if (shouldBeRunningIntake) {
			intake.set(-.4);
			// SmartDashboard.putNumber("INTAKE ON", 101);
		} else {
			intake.set(0);
			// SmartDashboard.putNumber("INTAKE OFF", 101);
		}


	}
	*/
	private void checkClimberState(){
		//CLIMBER LOGIC
		climberRight.set(joyStickRight.getY());
		climberLeft.set(joyStickRight.getY());
				
	} 
	
	private void toggleElevator() {
		
		// ELEVATOR
		if (logitechX2.get()) {
			if (!wasPressedLogitechX) {
				shouldBeRunningElevator = !shouldBeRunningElevator;
			}
			wasPressedLogitechX = true;
		} else {
			wasPressedLogitechX = false;
		}

		if (shouldBeRunningElevator) {
			elevator.set(-.3);
			SmartDashboard.putNumber("ELEVATOR ON", 101);
		} else {
			elevator.set(0);
			SmartDashboard.putNumber("ELEVATOR OFF", 101);
		}
	}
	
	private void toggleResExpansion(){
		if (logitechY2.get()) {
			if (!wasPressedLogitechB) {
				shouldBeRunningGearTray = !shouldBeRunningGearTray;
			}
			wasPressedLogitechB = true;
		} else {
			wasPressedLogitechB = false;
		}

		if (shouldBeRunningGearTray) {
			reservoirCylinder.set(DoubleSolenoid.Value.kReverse);
		} else {
			reservoirCylinder.set(DoubleSolenoid.Value.kForward);
		}
	} 
	public void shifterDelay(){
		int cycleCounterTele = 0;
		while(cycleCounterTele < 20){
			driveMotors(.3,-.3);
			cycleCounterTele++;
		}
	}
	/*
	private void toggleShifter() {

		if (logitechRightStickPress.get()) {
			if (!wasPressedRightStick) {
				
				shouldBeRunningShifter = !shouldBeRunningShifter;
				shifterDelay();
			}
			wasPressedRightStick = true;
		} else {
			wasPressedRightStick = false;
		}

		if (shouldBeRunningShifter) {
			shifterCylinder.set(DoubleSolenoid.Value.kForward);
			
		} else {
			shifterCylinder.set(DoubleSolenoid.Value.kReverse);
			
		}
		
	} 
	*/
	private void checkShift(){
		if(logitechRightBumper.get()){
			shifterDelay();
			shifterCylinder.set(DoubleSolenoid.Value.kForward);
		}
		if(logitechLeftBumper.get()){
			shifterDelay();
			shifterCylinder.set(DoubleSolenoid.Value.kReverse);
		}
	}
	/*
	private void singleStickArcade() {
		frontLeftDriveMotor.set(joyStickLeft.getX() - joyStickLeft.getY());
		frontRightDriveMotor.set(joyStickLeft.getX() + joyStickLeft.getY());
		backLeftDriveMotor.set(joyStickLeft.getX() - joyStickLeft.getY());
		backRightDriveMotor.set(joyStickLeft.getX() + joyStickLeft.getY());
	}
*/
	// SINGLE STICK DRIVE METHOD
	private void driveMotors(double speedLeftDM, double speedRightDM) {
		// System.out.println("Command: " + speedLeftDM);
		frontLeftDriveMotor.set(-speedLeftDM);
		backLeftDriveMotor.set(-speedLeftDM);
		frontRightDriveMotor.set(speedRightDM);
		backRightDriveMotor.set(speedRightDM);
	}

	// 2 STICK DRIVE METHOD
	private void arcadeDrive(double throttle, double turn) {
		driveMotors((throttle + turn), (throttle - turn));
	}
	
    private void turnMacro(float degrees){
    	ahrs.reset();
    	float nowRot = (float) ahrs.getAngle();
    	double outputDirection;
    	double outputPower;
    	while(degrees+5 > nowRot && nowRot > degrees-5){
    		if(nowRot > degrees){
    			outputDirection = 1;
    		}else{
    			outputDirection = -1;
    		}
    		outputPower = outputDirection*(((nowRot-degrees)/200)+.1);
    		driveMotors(outputPower,-outputPower);
    		nowRot = (float) ahrs.getAngle();
    	}
    }
    
    private void driveStraightFeet(float feet){
    	ahrs.reset();
    	float nowRot = (float) ahrs.getAngle();
    	double outputDirection;
    	double outputPower;
    	//double currentLocation;
    	if(distance < feet){
    		//driveMotors(.5, -.5);
    		while(5.0 > nowRot && nowRot > -5.0){
        		if(nowRot > 0){
        			outputDirection = 1;
        		}else{
        			outputDirection = -1;
        		}
        		
        		outputPower = outputDirection*(((0-nowRot)/200)+.1);
        		driveMotors(outputPower + 0.8 ,outputPower + -0.8);
        		nowRot = (float) ahrs.getAngle();
        	}
    	}
    	else{
    		driveMotors(0,0);
    	}
    }
    /*
    private void rotationMacro(){
    	if(logitechStart.get()){
			if(!wasPressedStart){
				macroPos = (float) ahrs.getAngle();
				shouldBeRunningAutoTurn = !shouldBeRunningAutoTurn;
			}
			wasPressedStart = true;
			System.out.println("yah boi be on");
		}else{
			wasPressedStart = false;
		}
    	if(shouldBeRunningSwitch){
    		turnMacro(30);
    	}else{
    		
    	}
    }
	*/
	// CORRECTION METHOD. WE USE THE VALUE QUARTERNION Z FOR ROTATIONAL
	// POSTITIONING
    //Spectre says HI 
	private void correct() {
		float nowRot = (float) ahrs.getAngle();
		System.out.println(rotationPos);
		System.out.println(nowRot);
		if (nowRot >= rotationPos + 10) {
			driveMotors(-.5,-.5);
		}
		if (nowRot <= rotationPos - 10) {
			driveMotors(.5,.5);
		}
    }

    public void encoderMacro(float encValue){
    	rotationCount = 0;
    	if(rotationCount < encValue){
			driveMotors(1, 1);
		}
    }
    
    public void encoderCreep(float encValue){
    	while(rotationCount >= 0.9 * encValue && rotationCount < encValue){
			driveMotors(0.3, 0.3);
		}
    	driveMotors(.5,.5);
    }

    public void gotoBoilerLeftWhileMiddlePosition(){
    	encoderMacro(000);
    	turnMacro(-90);
    	encoderMacro(000);
    	turnMacro(-45);
    	encoderCreep(000);
    }

    public void gotoBoilerRightWhileMiddlePosition(){
    	encoderMacro(000);
    	turnMacro(90);
    	encoderMacro(000);
    	turnMacro(45);
    	encoderCreep(000);
    }
    
    public void gotoBoilerLeftWhileLeftPosition(){
    	encoderMacro(000);
    	turnMacro(-90);
    	encoderMacro(000);
    	turnMacro(-45);
    	encoderCreep(000);
    }

    public void gotoBoilerRightWhileRightPosition(){
    	encoderMacro(000);
    	turnMacro(90);
    	encoderMacro(000);
    	turnMacro(45);
    	encoderCreep(000);
    }
    
    public void gotoLoadingRightWhileMiddlePosition(){
    	encoderMacro(000);
    	turnMacro(90);
    	encoderMacro(000);
    	turnMacro(45);
    	encoderCreep(000);
    }
    
    public void gotoLoadingLeftWhileMiddlePosition(){
    	encoderMacro(000);
    	turnMacro(-90);
    	encoderMacro(000);
    	turnMacro(-45);
    	encoderCreep(000);
    }
    
    public void gotoLoadingRightWhileRightPosition(){
    	encoderMacro(000);
    	turnMacro(90);
    	encoderMacro(000);
    	turnMacro(45);
    	encoderCreep(000);
    }

    public void gotoBoilerLeftFromMiddleGear(){  //Priority
    	encoderMacro(-000);
    	turnMacro(-120);
    	encoderMacro(000);
    }
    
    public void gotoLoadingLeftWhileLeftPosition(){
		encoderMacro(000);
		turnMacro(-90);
		encoderMacro(000);
		turnMacro(-45);
		encoderCreep(000);
	}
	public void gotoBoilerRightFromMiddleGear(){  //Priority
    	encoderMacro(-000);
    	turnMacro(120);
    	encoderMacro(000);
    }
    
    public void gotoLoadingRightFromMiddleGear(){
    	encoderMacro(-000);
    	turnMacro(120);
    	encoderMacro(000);
    }
    
    public void gotoLoadingLeftFromMiddleGear(){
    	encoderMacro(-000);
    	turnMacro(-120);
    	encoderMacro(000);
    }
    
    public void gearMiddle(){
    	encoderMacro(000);
    	encoderCreep(000);
    }

    public void gearLeftWhileLoading(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    	encoderCreep(000);
    }

    public void returnGearLeftWhileLoading(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    }
    
    public void gearRightWhileLoading(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    	encoderCreep(000);
    }
    
    public void returnGearRightWhileLoading(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    }

    public void gearLeftWhileBoiler(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    	encoderCreep(000);
    }
    
    public void returnGearLeftWhileBoiler(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    }

    public void gearRightWhileBoiler(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    	encoderCreep(000);
    }

    public void returnGearRightWhileBoiler(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    }

    public void hopperWhileBoilerLeft(){
    	encoderMacro(-000);
    	turnMacro(135);
    	encoderMacro(000);
    	turnMacro(-45);
    	encoderMacro(000);
    	turnMacro(45);
    }

    public void hopperWhileBoilerRight(){
    	encoderMacro(-000);
    	turnMacro(-135);
    	encoderMacro(000);
    	turnMacro(45);
    	encoderMacro(000);
    	turnMacro(-45);
    }

    public void hopperWhileLoadingLeft(){
    	encoderMacro(-000);
    	turnMacro(135);
    	encoderMacro(000);
    	turnMacro(-45);
    	encoderMacro(000);
    	turnMacro(45);
    }
    
    public void hopperWhileLoadingRight(){
    	encoderMacro(-000);
    	turnMacro(-135);
    	encoderMacro(000);
    	turnMacro(45);
    	encoderMacro(000);
    	turnMacro(-45);
    }

    public void returnHopperWhileBoilerLeft(){
    	encoderMacro(-000);
    	Timer.delay(1);
    	turnMacro(180);
    	encoderMacro(000);
    	turnMacro(45);
    	encoderCreep(000);
    }

    public void returnHopperWhileBoilerRight(){
    	encoderMacro(-000);
    	Timer.delay(1);
    	turnMacro(180);
    	encoderMacro(000);
    	turnMacro(-45);
    	encoderCreep(000);
    }

    public void returnHopperWhileLoadingLeft(){
    	encoderMacro(-000);
    	Timer.delay(1);
    	turnMacro(180);
    	encoderMacro(000);
    	turnMacro(45);
    	encoderCreep(000);
    }
    
    public void returnHopperWhileLoadingRight(){
    	encoderMacro(-000);
    	Timer.delay(1);
    	turnMacro(180);
    	encoderMacro(000);
    	turnMacro(-45);
    	encoderCreep(000);
    }

    /*public void shootAutonomous(double shootTime){
    	if(rotationRate >= 19000){
			spinUpComplete = true;
		} else {
			spinUpComplete = false;
		}
		
		if(!spinUpComplete){
			shooterRight.set(.62);
			shooterLeft.set(-.62);
		} else if (shouldBeRunningShooter && spinUpComplete) {
			rotationRate = shooterRightEnc.getRate();
			shooterSpeedCorrection = (19200-rotationRate)/5000;   //.62 is 19200, .65 is 20000, .61 is 19000
			shooterRight.set(.65+shooterSpeedCorrection);
			shooterLeft.set(-.65+shooterSpeedCorrection);
			System.out.println("Rotation Rate: " + rotationRate);
			System.out.println("Power Correction: " + shooterSpeedCorrection);
			
		} else {
			shooterRight.set(0);
		    shooterLeft.set(0);
		}
    }*/


    public void robotInit() {

		oi = new Controls();
		chooser = new SendableChooser();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		chooser.addObject("My Auto", "My Auto");
		SmartDashboard.putData("Auto mode", chooser);
		System.out.println(SmartDashboard.getBoolean("DB/Button 0", false));
		
		autoMode = RobotStates.noStringNoMove;

		// Motor port instantiating
		frontLeftDriveMotor = new Victor(Map.frontLeftDriveMotor);
		frontRightDriveMotor = new Victor(Map.frontRightDriveMotor);
		backLeftDriveMotor = new Victor(Map.backLeftDriveMotor);
		backRightDriveMotor = new Victor(Map.backRightDriveMotor);

		// Accessory motors
		intake = new Victor(Map.intakeMotor);
		
		climberLeft = new Victor(Map.leftClimberMotor);
		climberRight = new Victor(Map.rightClimberMotor);
		
		elevator = new Victor(Map.elevatorMotor);

		drive = new Encoder(
			Map.driveEncoderChannelA,
			Map.driveEncoderChannelB,
			true,
			Encoder.EncodingType.k4X
		);
		
		drive.setMaxPeriod(1);
		drive.setDistancePerPulse(36);
		drive.setMinRate(10);
		drive.setSamplesToAverage(32);


		joyStickLeft = new Joystick(Map.joystickDriverSlot);
		joyStickRight = new Joystick(Map.joystickManipulatorSlot);

		// BUTTON MAPPING. REASON ITS HERE IS BECAUSE IT WAS WRONG AND THESE ARE
		// THE CORRECT VALUES
		// back button 9
		// left stick 11
		// right stick 12
		// start 10
		// lefttrigger 7
		// righttrigger 8
		// leftbumper 5
		// rightbumper 6
		// y 4
		// b 3
		// a 2
		// x 1

		logitechY = new JoystickButton(joyStickLeft, 4);
		logitechA = new JoystickButton(joyStickLeft, 2);
		logitechX = new JoystickButton(joyStickLeft, 1);
		logitechB = new JoystickButton(joyStickLeft, 3);
		logitechLeftBumper = new JoystickButton(joyStickLeft, 5);
		logitechRightBumper = new JoystickButton(joyStickLeft, 6);
		logitechLeftStickPress = new JoystickButton(joyStickLeft, 11);
		logitechRightStickPress = new JoystickButton(joyStickLeft,12);
		logitechStart = new JoystickButton(joyStickLeft, 10);
		logitechBack = new JoystickButton(joyStickLeft, 9);
		logitechRightTrigger = new JoystickButton(joyStickLeft, 8);
		logitechLeftTrigger = new JoystickButton(joyStickLeft,7);
		
		logitechY2 = new JoystickButton(joyStickRight, 4);
		logitechA2= new JoystickButton(joyStickRight, 2);
		logitechX2 = new JoystickButton(joyStickRight, 1);
		logitechB2 = new JoystickButton(joyStickRight, 3);
		logitechLeftBumper2 = new JoystickButton(joyStickRight, 5);
		logitechRightBumper2 = new JoystickButton(joyStickRight, 6);
		logitechLeftStickPress2 = new JoystickButton(joyStickRight, 11);
		logitechRightStickPress2 = new JoystickButton(joyStickRight,12);
		logitechStart2 = new JoystickButton(joyStickRight, 10);
		logitechBack2 = new JoystickButton(joyStickRight, 9);
		logitechRightTrigger2 = new JoystickButton(joyStickRight, 8);
		logitechLeftTrigger2 = new JoystickButton(joyStickRight,7);

		
		shifterCylinder = new DoubleSolenoid(Map.shifterForwardChannel, Map.shifterBackwardChannel);
		shifterCylinder.set(DoubleSolenoid.Value.kForward);
		
		reservoirCylinder = new DoubleSolenoid(Map.reservoirForwardChannel, Map.reservoirBackwardChannel);
		reservoirCylinder.set(DoubleSolenoid.Value.kForward);

		
		// Boolean Toggle Switch
		shouldBeRunningSwitch = false;
		wasPressedLeftStick = false;
		shouldBeRunningShooter = false;
		wasPressedRightBumper = false;
		shouldBeRunningIntake = false;
		wasPressedLeftBumper = false;
		shouldBeRunningClimberUp = false;
		wasPressedLogitechA = false;
		shouldBeRunningClimberDown = false;
		wasPressedLogitechY = false;
		shouldBeRunningElevator = false;
		wasPressedLogitechX = false;
		shouldBeRunningGearTray = false;
		wasPressedLogitechB = false;
		
		spinUpComplete = false;

		wasPressedBackButton = false;
		shouldBeRunningCorrect = false;
		
		shouldBeRunningShifter = false;
		wasPressedRightStick = false;

		rotationPos = 0;
		
		driveState = true;
		
		
		// NavX instantiation
		try {
			ahrs = new AHRS(SerialPort.Port.kUSB);
			//ahrs = new AHRS(I2C.Port.kMXP); //WE WILL NEED I2C IN THE FUTURE.
			// RIGHT NOW WE WILL STICK WITH USB
		} catch (RuntimeException ex) {
			System.out.println("NavX instantiation error");
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		
		SmartDashboard.putString("DB/String 0", "left, right, middle");
		SmartDashboard.putString("DB/String 1", "red, blue");
		SmartDashboard.putString("DB/String 2", "boiler, loading");
		SmartDashboard.putString("DB/String 3", "Cross baseline?");
		SmartDashboard.putString("DB/String 4", "Where to place gear?");
		SmartDashboard.putString("DB/String 5", "Shoot before hopper?");
		SmartDashboard.putString("DB/String 6", "Shoot after hopper?");
		SmartDashboard.putString("DB/String 7", "Pickup at Hopper?");
		
	}

	private void dashboardDisplay() {
		if (isOperatorControl() && isEnabled()) {

			SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());

			/* Connectivity Debugging Support */
			SmartDashboard.putNumber("IMU_Byte_Count", ahrs.getByteCount());
			SmartDashboard.putNumber("IMU_Update_Count", ahrs.getUpdateCount());
		}
	}

	public void disabledInit() {
		
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		
	}

	public void autonomousInit() {
	    autonomousCommand = (Command) chooser.getSelected();
	    
		botPosition = SmartDashboard.getString("DB/String 0", "Is Bot at loading, middle, or boiler?");
		allianceColor = SmartDashboard.getString("DB/String 1", "Alliance Color red or blue?");
		chooseBoilerOrLoading = SmartDashboard.getString("DB/String 2", "Going boiler or loading side or nil?");
		baselineCross = SmartDashboard.getString("DB/String 3", "Only Cross Baseline? yes or no");
		gearPlacement = SmartDashboard.getString("DB/String 4", "Place gear right, left, middle, or nil?");
		shoot = SmartDashboard.getString("DB/String 5", "Shoot Before Hopper Pickup? yes or no");
		shootAfterHopper = SmartDashboard.getString("DB/String 6", "Shoot After Hopper Pickup? yes or no");
		hopperPickup = SmartDashboard.getString("DB/String 7", "Pickup balls from hopper? yes or no");
		
		
		shifterCylinder.set(DoubleSolenoid.Value.kReverse);
		reservoirCylinder.set(DoubleSolenoid.Value.kReverse);
		
		cycleCounter = 0;
		driveMotors(0, 0);
	   //autoSelecter = SmartDashboard.getNumber("DB/Slider 0", 0.0);
	    
	 
		//**************DEFAULT CODE IN CASE OF AN L**************
		if((botPosition.equalsIgnoreCase("test") &&										
				(allianceColor.equalsIgnoreCase(null) || allianceColor.equalsIgnoreCase(null)) &&
				chooseBoilerOrLoading.equalsIgnoreCase(null) &&
				baselineCross.equalsIgnoreCase(null) &&
				gearPlacement.equalsIgnoreCase(null) &&
				shoot.equalsIgnoreCase(null) &&
				shootAfterHopper.equalsIgnoreCase(null) &&
				hopperPickup.equalsIgnoreCase(null))){	
			autoMode = RobotStates.test;
		}
		if((botPosition.equalsIgnoreCase(null) &&										
				(allianceColor.equalsIgnoreCase(null) || allianceColor.equalsIgnoreCase(null)) &&
				chooseBoilerOrLoading.equalsIgnoreCase(null) &&
				baselineCross.equalsIgnoreCase(null) &&
				gearPlacement.equalsIgnoreCase(null) &&
				shoot.equalsIgnoreCase(null) &&
				shootAfterHopper.equalsIgnoreCase(null) &&
				hopperPickup.equalsIgnoreCase(null))){	
			autoMode = RobotStates.noStringNoMove;
		}
		if((botPosition.equalsIgnoreCase("right") || botPosition.equalsIgnoreCase("left")) &&			
				(allianceColor.equalsIgnoreCase("blue") || allianceColor.equalsIgnoreCase("red")) &&
				(chooseBoilerOrLoading.equalsIgnoreCase("boiler") || chooseBoilerOrLoading.equalsIgnoreCase("loading")) &&
				baselineCross.equalsIgnoreCase("yes") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){	
			System.out.println("auto changed");
			autoMode = RobotStates.baseline;
		}
		if((botPosition.equalsIgnoreCase("middle") &&										
				(allianceColor.equalsIgnoreCase("blue") || allianceColor.equalsIgnoreCase("red")) &&
				chooseBoilerOrLoading.equalsIgnoreCase("nil") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no"))){	
			autoMode = RobotStates.gearMiddle;
		}
		
		//**************BOILER-BASED FUNCTIONS**************
		
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.shootOnlyBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.shootOnlyBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.shootOnlyBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.shootOnlyBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperOnlyBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperOnlyBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperOnlyBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperOnlyBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearShootBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearShootBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperShootBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperShootBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperShootBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperShootBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.shootHopperShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.shootHopperShootBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.shootHopperShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.shootHopperShootBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoGearMiddleBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoGearMiddleBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoBoilerRightWhileMiddle;
		}
		
		//**************LOADING-BASED FUNCTIONS**************
		
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.loadingOnlyLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.loadingOnlyLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.loadingOnlyRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.loadingOnlyRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleLoadingLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleLoadingRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearLoadingLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearLoadingLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearLoadingRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearLoadingRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperLoadingLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperLoadingLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperLoadingRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperLoadingRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperLoadingLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperLoadingRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperLoadingLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperLoadingLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperLoadingRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperLoadingRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("turn") &&
				allianceColor.equalsIgnoreCase("turn") &&
				chooseBoilerOrLoading.equalsIgnoreCase("turn") &&
				baselineCross.equalsIgnoreCase("turn") &&
				gearPlacement.equalsIgnoreCase("turn") &&
				shoot.equalsIgnoreCase("turn") &&
				shootAfterHopper.equalsIgnoreCase("turn") &&
				hopperPickup.equalsIgnoreCase("turn")){
			autoMode = RobotStates.turnMacroTest;
		}
	}

	
	public void autonomousPeriodic() {
		/*NOTE: the notions of the method driveMotors are to be paired with encoder data.
		 * Make sure to pair!!!!!!!
		 * 
		 *NOTE: The hopper only methods for boiler side and loading side return from the hopper. If a "hopperStay" auton sequence is needed,
		 *contact Sam Sidhu ASAP 
		 *
		 */Scheduler.getInstance().run();
		 
		    cycleCounter++;
		 
		 
			distance = drive.getDistance();
		
			rotationCountForDrive = drive.get();
			rotationRateForDrive = drive.getRate();
		System.out.println(autoMode);
		switch(autoMode){
			case baseline: //Cross baseline
				encoderMacro(250);
			 
				if(cycleCounter < 250){
					driveMotors(1, 1);
				}
			
				break;
			case gearMiddle:
				gearMiddle();
				
				if(cycleCounter < 100){
					driveMotors(1, 1);
				}else if(cycleCounter < 350){
					driveMotors(0.3, 0.3);
				}else if(cycleCounter < 450){
					reservoirCylinder.set(DoubleSolenoid.Value.kForward);
					driveMotors(0, 0);
				}
	
				break;
			case gearMiddleBoilerLeft:
				gearMiddle();
				gotoBoilerLeftFromMiddleGear();
				break;
			case gearMiddleBoilerRight:
				gearMiddle();
				gotoBoilerRightFromMiddleGear();
				break;
			case gearBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				gearLeftWhileBoiler();
				break;
			case gearBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				gearLeftWhileBoiler();
				break;
			case gearBoilerRight:
				gotoBoilerRightWhileRightPosition();
				gearRightWhileBoiler();
				break;
			case gearBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				gearRightWhileBoiler();
				break;
			case shootOnlyBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				//shootAutonomous(0);
				break;
			case shootOnlyBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				//shootAutonomous(0);
				break;
			case shootOnlyBoilerRight:
				gotoBoilerRightWhileRightPosition();
				//shootAutonomous(0);
				break;
			case shootOnlyBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				//shootAutonomous(0);
				break;
			case hopperOnlyBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				break;
			case hopperOnlyBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				break;
			case hopperOnlyBoilerRight:
				gotoBoilerRightWhileRightPosition();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				break;
			case hopperOnlyBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				break;
			case gearMiddleShootBoilerLeft: //Priority
				gearMiddle();
				gotoBoilerLeftFromMiddleGear();
				//shootAutonomous(0);
				break;
			case gearMiddleShootBoilerRight: //Priority
				gearMiddle();
				gotoBoilerRightFromMiddleGear();
				//shootAutonomous(0);
				break;
			case gearShootBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				//shootAutonomous(0);
				break;
			case gearShootBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				//shootAutonomous(0);
				break;
			case gearShootBoilerRight:
				gotoBoilerRightWhileRightPosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				//shootAutonomous(0);
				break;
			case gearShootBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				//shootAutonomous(0);
				break;
			case hopperShootBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case hopperShootBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case hopperShootBoilerRight:
				gotoBoilerRightWhileRightPosition();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case hopperShootBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case gearMiddleHopperBoilerLeft:
				gearMiddle();
				gotoBoilerLeftFromMiddleGear();
				hopperWhileBoilerLeft();
				break;
			case gearMiddleHopperBoilerRight:
				gearMiddle();
				gotoBoilerRightFromMiddleGear();
				hopperWhileBoilerRight();
				break;
			case gearHopperBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				hopperWhileBoilerLeft();
				break;
			case gearHopperBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				hopperWhileBoilerLeft();
				break;
			case gearHopperBoilerRight:
				gotoBoilerRightWhileRightPosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				hopperWhileBoilerRight();
				break;
			case gearHopperBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				hopperWhileBoilerRight();
				break;
			case gearMiddleHopperShootBoilerLeft:
				gearMiddle();
				gotoBoilerLeftFromMiddleGear();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case gearMiddleHopperShootBoilerRight:
				gearMiddle();
				gotoBoilerRightFromMiddleGear();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case gearHopperShootBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case gearHopperShootBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case gearHopperShootBoilerRight:
				gotoBoilerRightWhileRightPosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case gearHopperShootBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case shootHopperShootBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				//shootAutonomous(0);
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case shootHopperShootBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				//shootAutonomous(0);
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case shootHopperShootBoilerRight:
				gotoBoilerRightWhileRightPosition();
				//shootAutonomous(0);
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case shootHopperShootBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				//shootAutonomous(0);
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case ultimateAutoGearMiddleBoilerLeft:
				gearMiddle();
				gotoBoilerLeftFromMiddleGear();
				//shootAutonomous(0);
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case ultimateAutoGearMiddleBoilerRight:
				gearMiddle();
				gotoBoilerRightFromMiddleGear();
				//shootAutonomous(0);
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case ultimateAutoBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				//shootAutonomous(0);
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case ultimateAutoBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				//shootAutonomous(0);
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case ultimateAutoBoilerRight:
				gotoBoilerRightWhileRightPosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				//shootAutonomous(0);
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case ultimateAutoBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				//shootAutonomous(0);
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case loadingOnlyLeft:
				gotoLoadingLeftWhileLeftPosition();
				break;
			case loadingOnlyLeftWhileMiddle:
				gotoLoadingLeftWhileMiddlePosition();
				break;
			case loadingOnlyRight:
				gotoLoadingRightWhileRightPosition();
				break;
			case loadingOnlyRightWhileMiddle:
				gotoLoadingRightWhileMiddlePosition();
				break;
			case gearMiddleLoadingLeft:
				gearMiddle();
				gotoLoadingLeftFromMiddleGear();
				break;
			case gearMiddleLoadingRight:
				gearMiddle();
				gotoLoadingRightFromMiddleGear();
				break;
			case gearLoadingLeft:
				gotoLoadingLeftWhileLeftPosition();
				gearLeftWhileLoading();
				returnGearLeftWhileLoading();
				break;
			case gearLoadingLeftWhileMiddle:
				gotoLoadingLeftWhileMiddlePosition();
				gearLeftWhileLoading();
				returnGearLeftWhileLoading();
				break;
			case gearLoadingRight:
				gotoLoadingRightWhileRightPosition();
				gearRightWhileLoading();
				returnGearRightWhileLoading();
				break;
			case gearLoadingRightWhileMiddle:
				gotoLoadingRightWhileMiddlePosition();
				gearRightWhileLoading();
				returnGearRightWhileLoading();
				break;
			case hopperLoadingLeft:
				gotoLoadingLeftWhileLeftPosition();
				hopperWhileLoadingLeft();
				returnHopperWhileLoadingLeft();
				break;
			case hopperLoadingLeftWhileMiddle:
				gotoLoadingLeftWhileMiddlePosition();
				hopperWhileLoadingLeft();
				returnHopperWhileLoadingLeft();
				break;
			case hopperLoadingRight:
				gotoLoadingRightWhileRightPosition();
				hopperWhileLoadingRight();
				returnHopperWhileLoadingRight();
				break;
			case hopperLoadingRightWhileMiddle:
				gotoLoadingRightWhileMiddlePosition();
				hopperWhileLoadingRight();
				returnHopperWhileLoadingRight();
				break;
			case gearMiddleHopperLoadingLeft:
				gearMiddle();
				gotoLoadingLeftFromMiddleGear();
				hopperWhileLoadingLeft();
				returnHopperWhileLoadingLeft();
				break;
			case gearMiddleHopperLoadingRight:
				gearMiddle();
				gotoLoadingRightFromMiddleGear();
				hopperWhileLoadingRight();
				returnHopperWhileLoadingRight();
				break;
			case gearHopperLoadingLeft:
				gotoLoadingLeftWhileLeftPosition();
				gearLeftWhileLoading();
				returnGearLeftWhileLoading();
				hopperWhileLoadingLeft();
				returnHopperWhileLoadingLeft();
				break;
			case gearHopperLoadingLeftWhileMiddle:
				gotoLoadingLeftWhileMiddlePosition();
				gearLeftWhileLoading();
				returnGearLeftWhileLoading();
				hopperWhileLoadingLeft();
				returnHopperWhileLoadingLeft();
				break;
			case gearHopperLoadingRight:
				gotoLoadingRightWhileRightPosition();
				gearRightWhileLoading();
				returnGearRightWhileLoading();
				hopperWhileLoadingRight();
				returnHopperWhileLoadingRight();
				break;
			case gearHopperLoadingRightWhileMiddle:
				gotoLoadingRightWhileMiddlePosition();
				gearRightWhileLoading();
				returnGearRightWhileLoading();
				hopperWhileLoadingRight();
				returnHopperWhileLoadingRight();
				break;
			case turnMacroTest:
				turnMacro(30);
				break;
			case noStringNoMove:
				driveMotors(0,0);
				break;
			case test:
				encoderCreep(3735);
		}	
		
		
	}

	public void teleopInit() {
		distance = 0;
		
		rotationCountForDrive = 0;
		rotationRateForDrive = 0;
		
		//currentCycle = 0;
		dualStick();
	}

	public void teleopPeriodic() {

		Scheduler.getInstance().run();
		
		distance = drive.getDistance();
		
		rotationCountForDrive = drive.get();
		rotationRateForDrive = drive.getRate();
		
		System.out.println("************");
		System.out.println(distance);
		System.out.println(rotationCountForDrive);
		System.out.println(rotationRateForDrive);
		System.out.println("************");
		System.out.println("Angle: "+ ahrs.getAngle());
		System.out.println("Elevator Current: "+power.elevator());
		System.out.println("Intake Current: "+power.intake());
		System.out.println("Front Right Drive Current: "+ power.frontRightDrive());
		System.out.println("Back Right Drive Current: "+ power.backLeftDrive());
		System.out.println("Front Left Drive Current: "+ power.frontLeftDrive());
		System.out.println("Back Left Drive Current: "+ power.backLeftDrive());
		System.out.println("Climber 1 Current: "+ power.climber1());
		System.out.println("Climber 2 Current: "+ power.climber2());
		
		//System.out.println(shooterEncoder.getDistance());
		//System.out.println(shooterEncoder.get());
		
		if(logitechA2.get()){
			intakeOnOff(.5);         //INTAKE CODE 
			shouldBeRunningIntake = true;
		}
		else if(logitechB2.get()){
			intakeOnOff(0);
			shouldBeRunningIntake = false;
		}
		
		
		if(logitechX.get()){
			driveState = true;
		}
		if(logitechY.get()){
			driveState = false;           //STATE CHANGE CODE
		}
		
		if(driveState){
			dualStick();
		}
		else{
			slowMove(.5);
		}
		 
		double intake = power.intake();
		if (intake >= 28) {
			intakeOnOff(0);
			Timer.delay(0.25);
			if (intake >= 24 && intake <= 28) {
				intakeOnOff(-0.6);
			} else if (intake < 24) {
				intakeOnOff(-0.85);

			}
		}

		if (intake >= 28) {
			elevator.set(0);
			Timer.delay(0.25);
			if (intake >= 24 && intake <= 28) {
				elevator.set(-0.15);
			} else if (intake < 24) {
				elevator.set(-0.3);
			}
		}
		
		//switchDriveModes();  

		testForCorrectionMode();

		//toggleShooterMotor();

		//toggleIntake();
		
		checkClimberState();
		
		toggleElevator();

		toggleResExpansion();
		
		checkShift();
		
		//toggleShifter();
		
		//operatorControl();

	}

	public void testPeriodic() {
		LiveWindow.run();
		dashboardDisplay();
	}
}
