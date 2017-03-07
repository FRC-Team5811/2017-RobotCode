package org.usfirst.frc.team5811.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.usfirst.frc.team5811.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
//import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import com.kauailabs.navx.frc.AHRS;

import java.math.*;

public class Robot extends IterativeRobot {

	public static OI oi;
	
	Command autonomousCommand;
	SendableChooser chooser = new SendableChooser();

	// Controllers
	Joystick joyStickLeft;
	Joystick joyStickRight;

	// NavX
	AHRS ahrs;

	// Buttons
	JoystickButton logitechY;
	
	//random addition
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
	Spark shooterRight;
	Spark shooterLeft;
	Victor climberLeft;
	Victor climberRight;
	Victor elevator;
	
	DigitalOutput agitator;
	
	UsbCamera camera;

	// Encoder definitions and variables
	Encoder shooterRightEnc;
	Encoder drive;
	
	//Counter shooterEncoder;
	int rotationCount;
	double rotationRate;
	double shooterSpeedCorrection;
	boolean encDirection;
	boolean encIfStopped;
	double rotationPeriod;
	boolean spinUpComplete;
	
	boolean driveState;
	
	DigitalInput limitSwitch;
	
	
	int stateSeq;
	int rotationCountForDrive;
	double rotationRateForDrive;
	double distance;
	
	double autoSelecter;
	
	//Autonomous
	int cycleCounter;
	/*
	enum RobotStates{ 
		 
		
		//**************BASE FUNCTIONS**************
		
		//Baseline Only
		baseline,
		
		//Gear Base
		gearMiddle,
		
		//**************BOILER-BASED FUNCTIONS**************
		
		//Gear Only
		gearMiddleBoilerLeft,
		gearMiddleBoilerRight,
		gearBoilerLeft,
		gearBoilerLeftWhileMiddle,
		gearBoilerRight,
		gearBoilerRightWhileMiddle,
		
		//Shoot Only
		shootOnlyBoilerLeft,
		shootOnlyBoilerLeftWhileMiddle,
		shootOnlyBoilerRight,
		shootOnlyBoilerRightWhileMiddle,
		
		//Hopper Only
		hopperOnlyBoilerLeft,
		hopperOnlyBoilerLeftWhileMiddle,
		hopperOnlyBoilerRight,
		hopperOnlyBoilerRightWhileMiddle,
		
		//Gear and Shoot
		gearMiddleShootBoilerLeft, //Priority
		gearMiddleShootBoilerRight,  //Priority
		gearShootBoilerLeft,
		gearShootBoilerLeftWhileMiddle,
		gearShootBoilerRight,
		gearShootBoilerRightWhileMiddle,
		
		//Hopper then Shoot
		hopperShootBoilerLeft,
		hopperShootBoilerLeftWhileMiddle,
		hopperShootBoilerRight,
		hopperShootBoilerRightWhileMiddle,
		
		//Gear and Hopper
		gearMiddleHopperBoilerLeft,
		gearMiddleHopperBoilerRight,
		gearHopperBoilerLeft,
		gearHopperBoilerLeftWhileMiddle,
		gearHopperBoilerRight,
		gearHopperBoilerRightWhileMiddle,
		
		//Hopper, Shoot, Place Gear || Place Gear, Hopper, then Shoot (Hopper Shoot Gear Sequences)
		gearMiddleHopperShootBoilerLeft,
		gearMiddleHopperShootBoilerRight,
		gearHopperShootBoilerLeft,
		gearHopperShootBoilerLeftWhileMiddle,
		gearHopperShootBoilerRight,
		gearHopperShootBoilerRightWhileMiddle,

		//Shoot, Hopper, Shoot
		shootHopperShootBoilerLeft,
		shootHopperShootBoilerLeftWhileMiddle,
		shootHopperShootBoilerRight,
		shootHopperShootBoilerRightWhileMiddle,
		
		//Ultimate Autonomous
		ultimateAutoGearMiddleBoilerLeft,
		ultimateAutoGearMiddleBoilerRight,
		ultimateAutoBoilerLeft, //gear shoot hopper shoot boiler left
		ultimateAutoBoilerLeftWhileMiddle,
		ultimateAutoBoilerRight,  //gear shoot hopper shoot boiler right
		ultimateAutoBoilerRightWhileMiddle,
		
		
		//**************LOADING-BASED FUNCTIONS**************
		
		//Loading Only
		loadingOnlyLeft,
		loadingOnlyLeftWhileMiddle,
		loadingOnlyRight,
		loadingOnlyRightWhileMiddle,
		
		//Gear and Loading
		gearMiddleLoadingLeft,
		gearMiddleLoadingRight,
		gearLoadingLeft,
		gearLoadingLeftWhileMiddle,
		gearLoadingRight,
		gearLoadingRightWhileMiddle,
		
		//Hopper and Loading
		hopperLoadingLeft,
		hopperLoadingLeftWhileMiddle,
		hopperLoadingRight,
		hopperLoadingRightWhileMiddle,
		
		//Hopper and Gear
		gearMiddleHopperLoadingLeft,
		gearMiddleHopperLoadingRight,
		gearHopperLoadingLeft,
		gearHopperLoadingLeftWhileMiddle,
		gearHopperLoadingRight,
		gearHopperLoadingRightWhileMiddle,
		
		//**************************************************************
		turnMacroTest,
		noStringNoMove,
		
		test,
	}
	*/
	//RobotStates autoMode; 
	
	/*
	public Robot(RobotStates autoMode){
		this.autoMode = autoMode;
	}
    */
	/*
	String botPosition;
	String allianceColor;
	String chooseBoilerOrLoading;
	String baselineCross;
	String gearPlacement;
	String shoot;
	String shootAfterHopper;
	String hopperPickup;
	*/
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

	// Current logic
	double currentElevator;
	double currentIntake;
	double currentFrontRightDrive;
	double currentBackRightDrive;
	double currentFrontLeftDrive;
	double currentBackLeftDrive;
	double currentClimber1;
	double currentClimber2;
	//double currentCycle;
	double n;

	// A cylinder
	DoubleSolenoid shifterCylinder;
	DoubleSolenoid reservoirCylinder;

	// COMPRESSOR!!!
	Compressor compressor;

	//Camera
	
	
	// power distribution panel
	PowerDistributionPanel power = new PowerDistributionPanel();

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
			//joyStickLeft.setRumble(RumbleType.kLeftRumble, 1);
			//joyStickLeft.setRumble(RumbleType.kRightRumble, 1);
		} else {
			dualStick();
			//joyStickLeft.setRumble(RumbleType.kLeftRumble, 1);
			//joyStickLeft.setRumble(RumbleType.kRightRumble, 0);
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
	
	private void toggleShooterMotor() {
		
		// SHOOTER + BLOCK PNEUMATIC

		if (logitechRightBumper2.get()) {
			if (!wasPressedRightBumper) {
				shouldBeRunningShooter = !shouldBeRunningShooter;
			}
			wasPressedRightBumper = true;
		} else {
			wasPressedRightBumper = false;
		}

		if(rotationRate >= 19000){
			spinUpComplete = true;
		} else {
			spinUpComplete = false;
		}
		
		if(shouldBeRunningShooter && !spinUpComplete){
			shooterRight.set(.67);
			shooterLeft.set(-.67);
		
		} else if (shouldBeRunningShooter && spinUpComplete) {
			rotationRate = shooterRightEnc.getRate();
			shooterSpeedCorrection = (21300-rotationRate)/5000;   //.62 is 19200, .65 is 20000, .61 is 19000 , .70 s 22000 ,.67 is 21300
			shooterRight.set(.65+shooterSpeedCorrection);
			shooterLeft.set(-.65+shooterSpeedCorrection);
			System.out.println("Rotation Rate: " + rotationRate);
			System.out.println("Power Correction: " + shooterSpeedCorrection);
			
		} else {
			shooterRight.set(0);
		    shooterLeft.set(0);
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
			elevator.set(-.5);
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
	
	
	
    private boolean turnMacro(float degrees){
    	//ahrs.reset();
    	float nowRot = (float) ahrs.getAngle();
    	double outputDirection;
    	double outputPower;
    	System.out.println("current position: "+nowRot);
    	System.out.println("set degrees: "+degrees);
    	if(degrees+5 < nowRot || degrees-5 > nowRot){
    		if(nowRot > degrees){
    			outputDirection = -1;
    		}else{
    			outputDirection = 1;
    		}
    		outputPower = (outputDirection*-.16)+outputDirection*(((nowRot-degrees)/500));
    		//outputPower = outputDirection*.3;
    		driveMotors(outputPower,-outputPower);
    		System.out.println("moving");
    		System.out.println(outputPower);
    		return false;
    	}
    	else{
    		driveMotors(0,0);
    		System.out.println("not moving");
    		ahrs.reset();
    		degrees = nowRot;
    		drive.reset();
    		return true;
    	}
    	
    }
    
    private void driveStraightFeet(float feet){
    	//ahrs.reset();
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
		if (nowRot >= rotationPos + 5) {
			driveMotors(-.5,.5);
		}
		if (nowRot <= rotationPos - 5) {
			driveMotors(.5,-.5);
		}
    }

    public boolean encoderMacro(float encValue){
    	rotationCountForDrive = (int) drive.getDistance();
    	if(Math.abs(rotationCountForDrive) < encValue){
			driveMotors(.3, .3);
			return false;
			//System.out.println(encValue);
		//	System.out.println(Math.abs(rotationCountForDrive));
		}
    	else{
    		driveMotors(0,0);
    		return true;
    	}
    }
    
    public void encoderCreep(float encValue){
    	while(rotationCount >= 0.9 * encValue && rotationCount < encValue){
			driveMotors(0.3, 0.3);
		}
    	driveMotors(.5,.5);
    }



    public void shootAutonomous(double shootTime){
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
    }


    public void robotInit() {

		oi = new OI();
		chooser = new SendableChooser();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		chooser.addObject("My Auto", "My Auto");
		SmartDashboard.putData("Auto mode", chooser);
		System.out.println(SmartDashboard.getBoolean("DB/Button 0", false));

		// Motor port instantiating
		frontLeftDriveMotor = new Victor(9);
		frontRightDriveMotor = new Victor(4);
		backLeftDriveMotor = new Victor(8);
		backRightDriveMotor = new Victor(3);

		// Accessory motors
		intake = new Victor(2);
		shooterRight = new Spark(0);
		shooterLeft = new Spark(5);
		climberLeft = new Victor(1);
		climberRight = new Victor(7);
		elevator = new Victor(6);
		
		
		agitator = new DigitalOutput(4);

		// Encoder inits and instantiations
		shooterRightEnc = new Encoder(0, 1, true, Encoder.EncodingType.k1X);
		shooterRightEnc.setMaxPeriod(1);
		shooterRightEnc.setDistancePerPulse(36);
		shooterRightEnc.setMinRate(10);
		shooterRightEnc.setSamplesToAverage(32);
		
		drive = new Encoder(2,3, true, Encoder.EncodingType.k4X);
		drive.setMaxPeriod(1);
		drive.setDistancePerPulse(36);
		drive.setMinRate(10);
		drive.setSamplesToAverage(32);
		
		
		
		//shooterEncoder = new Counter(0);
		//shooterEncoder.setSemiPeriodMode(true);
		//rotationCount = shooterRightEnc.get();
		//rotationRate = shooterRightEnc.getRate();
		//encIfStopped = shooterRightEnc.getStopped();
		//encDirection = shooterRightEnc.getDirection();// since it is a boolean its
													// either 0 or 1 (obv)...not
													// sure which value is which
													// direction though

		joyStickLeft = new Joystick(0);
		joyStickRight = new Joystick(1);
		
		//camera = CameraServer.getInstance().startAutomaticCapture();

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
		// rightTrim = SmartDashboard.getNumber("DB/Slider 3", 1.0);
		// if(rightTrim == 0){ SmartDashboard.putNumber("DB/Slider 3", 1);
		// rightTrim = 1;}

		shifterCylinder = new DoubleSolenoid(2, 3);
		reservoirCylinder = new DoubleSolenoid(6, 7);
		shifterCylinder.set(DoubleSolenoid.Value.kForward);
		reservoirCylinder.set(DoubleSolenoid.Value.kForward);

		// compressor port init
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(false);

		// limit switch init
		// imitSwitch = new DigitalInput(1);


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
		
		stateSeq = 0;
		
		//currentCycle = 0;
		
		//limitSwitch = new DigitalInput(1);
		
		//logicError = SmartDashboard.getString("DB/String 6", "No error");
		//endgameAutonomous = SmartDashboard.getString("DB/String 8", "Hah....BASIC");
		
		// NavX instantiation
		try {
			ahrs = new AHRS(SerialPort.Port.kUSB);
			//ahrs = new AHRS(I2C.Port.kMXP); //WE WILL NEED I2C IN THE FUTURE.
			// RIGHT NOW WE WILL STICK WITH USB
		} catch (RuntimeException ex) {
			System.out.println("NavX instantiation error");
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
	
		
	}

	private void operatorControl() {
		if (isOperatorControl() && isEnabled()) {

			Timer.delay(0.020); /* wait for one motor update time period (50Hz) */

			/*
			 * boolean zero_yaw_pressed = stick.getTrigger(); if (
			 * zero_yaw_pressed ) { ahrs.zeroYaw(); }
			 */

			/* Display 6-axis Processed Angle Data */
			/*
			SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
			SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
			SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
			SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
			SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

			/* Display tilt-corrected, Magnetometer-based heading (requires */
			/* magnetometer calibration to be useful) */

			//SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

			/*
			 * Display 9-axis Heading (requires magnetometer calibration to be
			 * useful)
			 */
		//	SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

			/*
			 * These functions are compatible w/the WPI Gyro Class, providing a
			 * simple
			 */
			/* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

			SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
			//SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

			/*
			 * Display Processed Acceleration Data (Linear Acceleration, Motion
			 * Detect)
			 */
/*
			SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
			SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
			SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
			SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());
*/
			/*
			 * Display estimates of velocity/displacement. Note that these
			 * values are
			 */
			/*
			 * not expected to be accurate enough for estimating robot position
			 * on a
			 */
			/*
			 * FIRST FRC Robotics Field, due to accelerometer noise and the
			 * compounding
			 */
			/*
			 * of these errors due to single (velocity) integration and
			 * especially
			 */
			/* double (displacement) integration. */
/*
			SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
			SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
			SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
			SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());
*/
			/* Display Raw Gyro/Accelerometer/Magnetometer Values */
			/*
			 * NOTE: These values are not normally necessary, but are made
			 * available
			 */
			/*
			 * for advanced users. Before using this data, please consider
			 * whether
			 */
			/* the processed data (see above) will suit your needs. */
/*
			SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
			SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
			SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
			SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
			SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
			SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
			SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
			SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
			SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
			SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
			SmartDashboard.putNumber("IMU_Timestamp", ahrs.getLastSensorTimestamp());
*/
			/* Omnimount Yaw Axis Information */
			/*
			 * For more info, see
			 * http://navx-mxp.kauailabs.com/installation/omnimount
			 */
		//	AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
			/*
			SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
			SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

			/* Sensor Board Information */
			//SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());
			/* Quaternion Data */
			/*
			 * Quaternions are fascinating, and are the most compact
			 * representation of
			 */
			/*
			 * orientation data. All of the Yaw, Pitch and Roll Values can be
			 * derived
			 */
			/*
			 * from the Quaternions. If interested in motion processing,
			 * knowledge of
			 */
			
			/* Quaternions is highly recommended. */
			/*
			SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
			SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
			SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
			SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());

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
	    autoSelecter = SmartDashboard.getNumber("DB/Slider 0", 0.0);
	    /*

		*/
		
		shifterCylinder.set(DoubleSolenoid.Value.kReverse);
		reservoirCylinder.set(DoubleSolenoid.Value.kReverse);
		//rotationCountForDrive = 0;
		drive.reset();
		ahrs.reset();
		cycleCounter = 0;
		stateSeq = 0;
		driveMotors(0, 0);
	   //autoSelecter = SmartDashboard.getNumber("DB/Slider 0", 0.0);
	   
	  
	    
	 
		//**************DEFAULT CODE IN CASE OF AN L**************

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
		    
		    if(autoSelecter == 0.0){   //DRIVE FORWARD
		    	switch(stateSeq){
				case 0:
					System.out.println("stage 0");
					encoderMacro(3000);
					break;
		    	}
		    }
		    else if(autoSelecter == 1.0){  //TURN
		    	if(stateSeq ==0){
		    		
		    		System.out.println("stage 0");
		    		if(turnMacro(60)){
		    			System.out.println("incrementing");
		    			drive.reset();
		    			stateSeq++;
		    		}
		    	}
		    	if(stateSeq == 1){
		    		
		    		System.out.println("stage 1");
		    		if(encoderMacro(10000)){
		    			ahrs.reset();
		        		drive.reset();
		        		stateSeq++;
		    		}
		    	
		    	}
		    	System.out.println("stateSeq: "+stateSeq);
		    }
		 
		 
			distance = drive.getDistance();
		
			
			System.out.println(Math.abs(rotationCountForDrive));
			//System.out.println(encValue);
			//System.out.println(Math.abs(drive.getDistance()));
			System.out.println("Raw Angle: "+ahrs.getAngle());
			rotationRateForDrive = drive.getRate();
			
			System.out.println(autoSelecter);
	
		
		
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
		compressor.setClosedLoopControl(true);
		
		rotationCount = shooterRightEnc.get();
		rotationRate = shooterRightEnc.getRate();
		//double distance = shooterRightEnc.getDistance();
		boolean direction = shooterRightEnc.getDirection();
		boolean stopped = shooterRightEnc.getStopped();
		rotationPeriod = shooterRightEnc.getRaw();
		
		distance = drive.getDistance();
		
		rotationCountForDrive = drive.get();
		rotationRateForDrive = drive.getRate();
		
		System.out.println("************");
		System.out.println(distance);
		System.out.println(rotationCountForDrive);
		System.out.println(rotationRateForDrive);
		System.out.println("************");
		System.out.println("Angle: "+ahrs.getAngle());
		System.out.println("Elevator Current: "+currentElevator);
		System.out.println("Intake Current: "+currentIntake);
		System.out.println("Front Right Drive Current: "+currentFrontRightDrive);
		System.out.println("Back Right Drive Current: "+currentBackRightDrive);
		System.out.println("Front Left Drive Current: "+currentFrontLeftDrive);
		System.out.println("Back Left Drive Current: "+currentBackLeftDrive);
		System.out.println("Climber 1 Current: "+currentClimber1);
		System.out.println("Climber 2 Current: "+currentClimber2);
		
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
			slowMove(.35);
		}
		
		currentElevator = power.getCurrent(3);
		currentIntake = power.getCurrent(13);
		currentFrontRightDrive = power.getCurrent(15);
		currentBackRightDrive = power.getCurrent(14);
		currentFrontLeftDrive = power.getCurrent(0);
		currentBackLeftDrive = power.getCurrent(1);
		currentClimber1 = power.getCurrent(12);
		currentClimber2 = power.getCurrent(2);
		 
			if(currentIntake >= 28){
				intakeOnOff(0);
				Timer.delay(0.25);
				if(currentIntake >= 24 && currentIntake <= 28){
					intakeOnOff(-0.6);
				}else if(currentIntake < 24){
					intakeOnOff(-0.85);
					
				}
			}
	
			if(currentElevator >= 28){
				elevator.set(0);
				Timer.delay(0.25);
				if(currentElevator >= 24 && currentElevator <= 28){
					elevator.set(-0.15);
				}else if(currentElevator < 24){
					elevator.set(-0.3);
					
				}
			}
	
		
		//switchDriveModes();  

		testForCorrectionMode();

		toggleShooterMotor();

		//toggleIntake();
		
		checkClimberState();
		
		toggleElevator();

		toggleResExpansion();
		
		checkShift();
		
		agitator.set(false);
		
		//toggleShifter();
		
		//operatorControl();

	}

	public void testPeriodic() {
		LiveWindow.run();
		operatorControl();
	}
}
