package org.usfirst.frc.team5811.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.usfirst.frc.team5811.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DigitalOutput;

public class Robot extends IterativeRobot {

	public static OI oi;
	
	Command autonomousCommand;
	SendableChooser chooser = new SendableChooser();

	// Controllers
	Joystick joyStickLeft;
	Joystick joyStickRight;

	// NavX
	AHRS ahrs;
	
	//Outputs to Arduino
	DigitalOutput red = new DigitalOutput(0);
	DigitalOutput green = new DigitalOutput(1);
	DigitalOutput blue = new DigitalOutput(2);
	
	//Serial i2c
	I2C arduino;

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
	Spark shooterRight;
	Spark shooterLeft;
	Victor climberLeft;
	Victor climberRight;
	Victor elevator;
	
	//DigitalOutput agitator;
	
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
	//boolean shouldBeRunningSwitch;
	//boolean wasPressedLeftStick;
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
	private void dualStickEXP(){
		arcadeDrive(-joyStickLeft.getRawAxis(1)*Math.abs(joyStickLeft.getRawAxis(1)),joyStickLeft.getRawAxis(2)*Math.abs(joyStickLeft.getRawAxis(2)));
	}
	private void slowMove(double reduction){
		//shifterCylinder.set(DoubleSolenoid.Value.kReverse);
		arcadeDrive((-joyStickLeft.getRawAxis(1)*reduction), (joyStickLeft.getRawAxis(2)*reduction));
	}
	public void intakeOnOff(double speed){
		intake.set(speed);
	}
	public void resetAutoEncNavX(){
		ahrs.reset();
		drive.reset();
		rotationPos = 0;
		Timer.delay(.5);
		stateSeq++;
	}
	/*
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
	*/
	
	private void toggleShooterMotor() {
		// SHOOTER
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
	
	private void checkClimberState(){
		//CLIMBER LOGIC
		climberRight.set(Math.abs(joyStickRight.getY()));
		climberLeft.set(Math.abs(joyStickRight.getY()));
				
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

	private void checkShift(){
		if(logitechRightBumper.get()){
			//shifterDelay();
			shifterCylinder.set(DoubleSolenoid.Value.kForward);
			System.out.println("HIGH GEAR");
		}
		if(logitechLeftBumper.get()){
			//shifterDelay();
			shifterCylinder.set(DoubleSolenoid.Value.kReverse);
			System.out.println("LOW GEAR");
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
    	float nowRot = (float) ahrs.getAngle();
    	double outputDirection;
    	double outputPower;
    	System.out.println("current position: "+nowRot);
    	System.out.println("set degrees: "+degrees);
    	if(Math.abs(degrees)+1 < nowRot || Math.abs(degrees)-1 > nowRot){
    		if(nowRot > degrees){
    			outputDirection = -1;
    		}else{
    			outputDirection = 1;
    		}
    		outputPower = (outputDirection*-.3)+outputDirection*(((nowRot-Math.abs(degrees))/1000));
    		driveMotors(-outputPower,outputPower);
    		System.out.println("moving");
    		return false;
    	}
    	else{
    		driveMotors(0,0);
    		System.out.println("not moving");
    		//ahrs.reset();
    		degrees = nowRot;
    		//drive.reset();
    		return true;
    	}
    }
	// CORRECTION METHOD. WE USE THE VALUE getAngleFOR ROTATIONAL
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
			float nowRot = (float) ahrs.getAngle();
			float error = rotationPos-nowRot;
			System.out.println(rotationPos);
			System.out.println(nowRot);
			driveMotors(.30+error/10,.30-error/10);
			return false;
		}
    	else{
    		driveMotors(0,0);
    		return true;
    	}
    }
    public void shootAutonomous(){
	    	if(rotationRate >= 19000){
				spinUpComplete = true;
			} else {
				spinUpComplete = false;
			}
		if(!spinUpComplete){
			shooterRight.set(.62);
			shooterLeft.set(-.62);
			elevator.set(-.5);
		} else if (shouldBeRunningShooter && spinUpComplete) {
			rotationRate = shooterRightEnc.getRate();
			shooterSpeedCorrection = (19200-rotationRate)/5000;   //.62 is 19200, .65 is 20000, .61 is 19000
			shooterRight.set(.65+shooterSpeedCorrection);
			shooterLeft.set(-.65+shooterSpeedCorrection);
			elevator.set(-.5);
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
		
		arduino = new I2C(I2C.Port.kMXP, 58);
		//Outputs to Arduino
	
		
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
	
	//	agitator = new DigitalOutput(4);

		// Encoder inits and instantiations
		/*
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
		*/
		
		
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
		
		camera = CameraServer.getInstance().startAutomaticCapture();

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

		shifterCylinder = new DoubleSolenoid(2, 3);
		reservoirCylinder = new DoubleSolenoid(6, 7);
		shifterCylinder.set(DoubleSolenoid.Value.kForward);
		reservoirCylinder.set(DoubleSolenoid.Value.kForward);

		// compressor port init
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(false);
		
		// Boolean Toggle Switches
		//shouldBeRunningSwitch = false;
		//wasPressedLeftStick = false;
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
	    String autoDEF1 = SmartDashboard.getString("DB/String 0", "0 is GEAR MIDDLE");
	    String autoDEF2 = SmartDashboard.getString("DB/String 1", "1 is GEAR LEFT RED");
	    String autoDEF3 = SmartDashboard.getString("DB/String 2", "2 is GEAR RIGHT RED");
	    String autoDEF4 = SmartDashboard.getString("DB/String 3", "3 is GEAR LEFT BLUE");
	    String autoDEF5 = SmartDashboard.getString("DB/String 4", "4 is GEAR RIGHT BLUE");
	    
	    driveMotors(0, 0);
		shifterCylinder.set(DoubleSolenoid.Value.kReverse);
		reservoirCylinder.set(DoubleSolenoid.Value.kReverse);
		rotationPos =0;
		drive.reset();
		ahrs.reset();
		cycleCounter = 0;
		
		stateSeq = 0;
		Timer.delay(.5);
	}

	public void autonomousPeriodic() {
			Scheduler.getInstance().run();
		 
		    cycleCounter++;
		    
		    if(autoSelecter == 0.0){   //GEAR MIDDLE
			    switch(stateSeq){
			    case 0:
			    	if(encoderMacro(132500)){
			    		resetAutoEncNavX();
			    	}
			    	break;
			    case 1:
			    	driveMotors(0,0);
			    	break;
			    }
		    }
		    else if(autoSelecter == 1.0){ //HOPPER LEFT SIDE
		    	switch(stateSeq){
		    	case 0:
		    		if(encoderMacro(152500)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 1:
		    		if(turnMacro(60)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 2:
		    		if(encoderMacro(43000)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 3:
		    		driveMotors(0,0);
		    		break;
		    	}
		    }
		    else if(autoSelecter == 2.0){//BOILER RIGHT SIDE
		    	switch(stateSeq){
		    	case 0:
		    		if(encoderMacro(152500)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 1:
		    		if(turnMacro(300)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 2:
		    		if(encoderMacro(53700)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 3:
		    		driveMotors(0,0);
		    		break;
		    	}
		    }
		    else if(autoSelecter == 3.0){  //BOILER LEFT SIDE
		    	switch(stateSeq){
		    	case 0:
		    		if(encoderMacro(152500)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 1:
		    		if(turnMacro(60)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 2:
		    		if(encoderMacro(53700)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 3:
		    		driveMotors(0,0);
		    		break;
		    	}
		    }
		    else if(autoSelecter == 4.0){      //HOPPER RIGHT SIDE
		      	switch(stateSeq){
		    	case 0:
		    		if(encoderMacro(152500)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 1:
		    		if(turnMacro(300)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 2:
		    		if(encoderMacro(43000)){
		    			resetAutoEncNavX();
		    		}
		    		break;
		    	case 3:
		    		driveMotors(0,0);
		    		break;
		    	}
		    }
		 
		 
			distance = drive.getDistance();
		
			System.out.println("***************");
			System.out.println("Encoder pulse count: "+Math.abs(rotationCountForDrive));
			System.out.println("Raw Angle: "+ahrs.getAngle());
			System.out.println("Auto Mode: "+autoSelecter);
			System.out.println("stage: "+stateSeq);
			System.out.println("***************");
	}

	public void teleopInit() {
		distance = 0;
		
		rotationCountForDrive = 0;
		rotationRateForDrive = 0;
		shifterCylinder.set(DoubleSolenoid.Value.kReverse);
		
		//currentCycle = 0;
		dualStickEXP();
		
		//dualStick();
	}
	byte[] toSend = new byte[1];
	public void teleopPeriodic() {
		
		//arduino.transaction(dataToSend, sendSize, dataReceived, receiveSize)
		
		
		
		if(arduino.writeBulk(toSend)){
			System.out.println("didnt send");
		}
		
		Scheduler.getInstance().run(); 
		compressor.setClosedLoopControl(true);
		/*
		rotationCount = shooterRightEnc.get();
		rotationRate = shooterRightEnc.getRate();
		//double distance = shooterRightEnc.getDistance();
		boolean direction = shooterRightEnc.getDirection();
		boolean stopped = shooterRightEnc.getStopped();
		rotationPeriod = shooterRightEnc.getRaw();
		
		distance = drive.getDistance();
		
		rotationCountForDrive = drive.get();
		rotationRateForDrive = drive.getRate();
		*/
		System.out.println("************");
		//System.out.println(distance);
		System.out.println("Encoder: "+rotationCountForDrive);
		//System.out.println(rotationRateForDrive);
		System.out.println("************");
		//System.out.println("Angle: "+ahrs.getAngle());
		//System.out.println("Elevator Current: "+currentElevator);
		//System.out.println("Intake Current: "+currentIntake);
	//	System.out.println("Front Right Drive Current: "+currentFrontRightDrive);
		//System.out.print("      Back Right Drive Current: "+currentBackRightDrive);
		//System.out.print("      Front Left Drive Current: "+currentFrontLeftDrive);
		//System.out.println("      Back Left Drive Current: "+currentBackLeftDrive);
		//System.out.println("Climber 1 Current: "+currentClimber1);
		//System.out.println("Climber 2 Current: "+currentClimber2);
		//System.out.println("**************");
		//System.out.println(shooterEncoder.getDistance());
		//System.out.println("Shooter Count: "+shooterRightEnc.get());
		
		if(logitechA2.get()){
			intakeOnOff(.5);         //INTAKE CODE 
			shouldBeRunningIntake = true;
		}
		else if(logitechLeftTrigger2.get()){
			intakeOnOff(-.5);
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
			driveState = false;         //STATE CHANGE CODE
		}
		if(logitechA.get()){
			toSend[0] = 1;
		}
		else
		{
			toSend[0] = 2;
		}
		
		if(driveState){
			dualStickEXP();
			//dualStick();
			System.out.println("NORMAL DRIVE");
		}
		else{
			dualStick();
			//slowMove(.4);
			System.out.println("IN SLOW MODE");
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

		//testForCorrectionMode();

		toggleShooterMotor();
		
		checkClimberState();
		
		toggleElevator();

		toggleResExpansion();
		
		checkShift();
		if(shifterCylinder.get() == DoubleSolenoid.Value.kReverse){
			System.out.println("LOW GEAR");
		}
		if(shifterCylinder.get() == DoubleSolenoid.Value.kForward){
			System.out.println("HIGH GEAR");
		}
	
	}

	public void testPeriodic() {
		LiveWindow.run();
		operatorControl();
	}
}
