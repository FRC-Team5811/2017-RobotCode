package org.usfirst.frc.team5811.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.usfirst.frc.team5811.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import com.kauailabs.navx.frc.AHRS;

public class Robot extends IterativeRobot {

	public static OI oi;

	Command autonomousCommand;
	SendableChooser chooser = new SendableChooser();

	// Controllers
	Joystick joyStickLeft;
	
	// Joystick joyStickRight;
	Joystick logitech;

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
	JoystickButton logitechStart;
	JoystickButton logitechBack;

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

	// Encoder definitions and variables
	Encoder shooterRightEnc;
	//Counter shooterEncoder;
	int rotationCount;
	double rotationRate;
	double shooterSpeedCorrection;
	boolean encDirection;
	boolean encIfStopped;
	double rotationPeriod;
	boolean spinUpComplete;

	// misc
	// double intakePower;
	/*
	 * //Auto Intake values double autoSelecter; boolean releaseToggle; boolean
	 * raiseAfterRelease; boolean returnAfter; double turnTime; double
	 * turnPower; boolean shootBall;
	 * 
	 * //Intake current values int cycleCounter; int intakeCounter; boolean
	 * firstSpikeStarted; boolean firstSpikeFinished; boolean
	 * secondSpikeStarted; int secondIntakeCounter; double ultrasonicVoltage;
	 * int rightThreshold; int centerThreshold; int leftThreshold;
	 */
	
	//Autonomous
	int cycleCounter;
	enum RobotStates{ //#blessed because our dude Chris taught us the ways of the enumerations
		/*
		 * How to read: (sequence of methods [i.e. shootgear = shoot then place gear])
		 * + (location on field where interaction occurs [i.e. Boiler is at the Boiler])
		 * + (side of field, depends on alliance due to field being rotationally asymmetrical [i.e. Boiler Right is Red Alliance])
		*/ 
		
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
	}
	
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

	boolean shouldBeRunningCorrect;
	boolean wasPressedBackButton;
	
    boolean shouldBeRunningAutoTurn;
    boolean wasPressedStart;

	// Global Speed Values
	double leftSpeed;
	double rightSpeed;

	// Current logic
	double current;
	double n;

	// A cylinder
	DoubleSolenoid gearTrayCylinder;
	DoubleSolenoid ballBlockCylinder;

	// COMPRESSOR!!!
	Compressor compressor;

	// power distribution panel
	PowerDistributionPanel power = new PowerDistributionPanel();

	float rotationPos;
	float macroPos;
	
	private void switchDriveModes(){
		// SWITCHING BETWEEN DRIVE MODES
		if (logitechLeftStickPress.get()) {
			if (!wasPressedLeftStick) {
				shouldBeRunningSwitch = !shouldBeRunningSwitch;
			}
			wasPressedLeftStick = true;
			System.out.println("yah boi be on");
		} else {
			wasPressedLeftStick = false;
		}

		if (shouldBeRunningSwitch) {
			arcadeDrive(joyStickLeft.getRawAxis(0) + (joyStickLeft.getRawAxis(3) - joyStickLeft.getRawAxis(2)),
					joyStickLeft.getRawAxis(5));
			joyStickLeft.setRumble(RumbleType.kLeftRumble, 1);
			joyStickLeft.setRumble(RumbleType.kRightRumble, 1);
		} else {
			singleStickArcade();
			joyStickLeft.setRumble(RumbleType.kLeftRumble, 1);
			joyStickLeft.setRumble(RumbleType.kRightRumble, 0);
		}
	}
	
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

		if (logitechRightBumper.get()) {
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
			shooterRight.set(.62);
			shooterLeft.set(-.62);
		} else if (shouldBeRunningShooter && spinUpComplete) {
			rotationRate = shooterRightEnc.getRate();
			shooterSpeedCorrection = (19200-rotationRate)/5000;   //.62 is 19200, .65 is 20000, .61 is 19000
			shooterRight.set(.65+shooterSpeedCorrection);
			shooterLeft.set(-.65+shooterSpeedCorrection);
			System.out.println("Rotation Rate: " + rotationRate);
			System.out.println("Power Correction: " + shooterSpeedCorrection);
			ballBlockCylinder.set(DoubleSolenoid.Value.kReverse);
		} else {
			shooterRight.set(0);
			 shooterLeft.set(0);
			ballBlockCylinder.set(DoubleSolenoid.Value.kForward);
		}
		
	}
	
	private void toggleIntake() {
		
		// INTAKE MY DUDES

		if (logitechLeftBumper.get()) {
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
/*
	private void checkClimberState(){
		//CLIMBER LOGIC
		if (logitechA.get()) {
			// CLIMBER DOWN
			climber.set(-1);
		} else if(logitechY.get()) {
			// CLIMBER UP
			climber.set(1);
		}else{
			climber.set(0);
		}
				
	} 
	*/
	private void toggleElevator() {
		
		// ELEVATOR
		if (logitechX.get()) {
			if (!wasPressedLogitechX) {
				shouldBeRunningElevator = !shouldBeRunningElevator;
			}
			wasPressedLogitechX = true;
		} else {
			wasPressedLogitechX = false;
		}

		if (shouldBeRunningElevator) {
			elevator.set(1);
			SmartDashboard.putNumber("ELEVATOR ON", 101);
		} else {
			elevator.set(0);
			SmartDashboard.putNumber("ELEVATOR OFF", 101);
		}

		
	}
	
	private void toggleGearTray() {
		
		// GEAR TRAY

		if (logitechB.get()) {
			if (!wasPressedLogitechB) {
				shouldBeRunningGearTray = !shouldBeRunningGearTray;
			}
			wasPressedLogitechB = true;
		} else {
			wasPressedLogitechB = false;
		}

		if (shouldBeRunningGearTray) {
			gearTrayCylinder.set(DoubleSolenoid.Value.kForward);
			SmartDashboard.putNumber("GEAR TRAY OUT", 101);
		} else {
			gearTrayCylinder.set(DoubleSolenoid.Value.kReverse);
			SmartDashboard.putNumber("GEAR TRAY IN", 101);
		}

		
	} 
	
	private void singleStickArcade() {
		frontLeftDriveMotor.set(joyStickLeft.getX() - joyStickLeft.getY());
		frontRightDriveMotor.set(joyStickLeft.getX() + joyStickLeft.getY());
		backLeftDriveMotor.set(joyStickLeft.getX() - joyStickLeft.getY());
		backRightDriveMotor.set(joyStickLeft.getX() + joyStickLeft.getY());
	}

	// SINGLE STICK DRIVE METHOD
	private void driveMotors(double speedLeftDM, double speedRightDM) {
		// System.out.println("Command: " + speedLeftDM);
		frontLeftDriveMotor.set(speedLeftDM);
		backLeftDriveMotor.set(speedLeftDM);
		frontRightDriveMotor.set(-speedRightDM);
		backRightDriveMotor.set(-speedRightDM);
	}

	// 2 STICK DRIVE METHOD
	private void arcadeDrive(double throttle, double turn) {
		driveMotors((throttle + turn), (throttle - turn));
	}
	
    private void turnMacro(float degrees){
    	float nowRot = (float) ahrs.getAngle();
    	if((macroPos + degrees) > nowRot){
    		frontLeftDriveMotor.set(-.5);
			backLeftDriveMotor.set(-.5);
			frontRightDriveMotor.set(-.5);
			backRightDriveMotor.set(-.5);
    	}else{
    		frontLeftDriveMotor.set(0);
			backLeftDriveMotor.set(0);
			frontRightDriveMotor.set(0);
			backRightDriveMotor.set(0);
			wasPressedStart = false;
			shouldBeRunningAutoTurn = false;
    	}
    }
    
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
	
	// CORRECTION METHOD. WE USE THE VALUE QUARTERNION Z FOR ROTATIONAL
	// POSTITIONING
    //Spectre says HI 
	private void correct() {
		float nowRot = (float) ahrs.getAngle();
		System.out.println(rotationPos);
		System.out.println(nowRot);
		if (nowRot >= rotationPos + 10) {
			frontLeftDriveMotor.set(-.5);
			backLeftDriveMotor.set(-.5);
			frontRightDriveMotor.set(-.5);
			backRightDriveMotor.set(-.5);
		}
		if (nowRot <= rotationPos - 10) {
			frontLeftDriveMotor.set(.5);
			backLeftDriveMotor.set(.5);
			frontRightDriveMotor.set(.5);
			backRightDriveMotor.set(.5);
		}
    }
	
    public void correctSwitch(){
        if(logitechBack.get()){
        	if(!wasPressedBackButton){
        		rotationPos = (float) ahrs.getAngle();
        		shouldBeRunningCorrect = !shouldBeRunningCorrect;
        	}
        	wasPressedBackButton = true;
        }else{
        	wasPressedBackButton = false;
        }
        
        if(shouldBeRunningCorrect){
        	correct();
        	System.out.println("in correct mode");
        }else{
        	System.out.println("not in correct mode");
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
    
    public void gotoLoadingWhileLeftPosition(){
    	encoderMacro(000);
    	turnMacro(-90);
    	encoderMacro(000);
    	turnMacro(-45);
    	encoderCreep(000);
    }
    
    public void gotoLoadingWhileRightPosition(){
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
    	if(current < 000){
    		encoderMacro(0);
    	}
    }
    
    public void gotoBoilerRightFromMiddleGear(){  //Priority
    	encoderMacro(-000);
    	turnMacro(120);
    	encoderMacro(000);
    	if(current < 000){
    		encoderMacro(0);
    	}
    }
    
    public void gotoLoadingRightFromMiddleGear(){
    	encoderMacro(-000);
    	turnMacro(120);
    	encoderMacro(000);
    	if(current < 000){
    		encoderMacro(0);
    	}
    }
    
    public void gotoLoadingLeftFromMiddleGear(){
    	encoderMacro(-000);
    	turnMacro(-120);
    	encoderMacro(000);
    	if(current < 000){
    		encoderMacro(0);
    	}
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

    public void gearRightWhileLoading(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    	encoderCreep(000);
    }
    
    public void gearLeftWhileBoiler(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    	encoderCreep(000);
    }
    
    public void gearRightWhileBoiler(){
    	encoderMacro(-000);
    	turnMacro(180);
    	encoderMacro(000);
    	encoderCreep(000);
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

    public void shootAutonomous(double shootTime){
    	if(rotationRate >= 19000){
			spinUpComplete = true;
		} else {
			spinUpComplete = false;
		}
		
		if(!spinUpComplete){
			shooterRight.set(.61);
			shooterLeft.set(-.61);
		} else if (spinUpComplete) {
			rotationRate = shooterRightEnc.getRate();
			shooterSpeedCorrection = (19000-rotationRate)/5000;
			shooterRight.set(.61 + shooterSpeedCorrection);
			shooterLeft.set(-.61 + shooterSpeedCorrection);
			ballBlockCylinder.set(DoubleSolenoid.Value.kReverse);
			Timer.delay(shootTime);
			shooterRight.set(0);
			shooterLeft.set(0);
			ballBlockCylinder.set(DoubleSolenoid.Value.kForward);
		} 
    }
    
    public void driveSwitch(){
    	if(logitechLeftStickPress.get()){
			if(!wasPressedLeftStick){
				shouldBeRunningSwitch = !shouldBeRunningSwitch;
			}
			wasPressedLeftStick = true;
			System.out.println("yah boi be on");
		}else{
			wasPressedLeftStick = false;
		}
    	if(shouldBeRunningSwitch){
    		
    	//** MUST BE FIXED!!!!!
			arcadeDrive(joyStickLeft.getRawAxis(0)+(joyStickLeft.getRawAxis(3)-joyStickLeft.getRawAxis(2)),joyStickLeft.getRawAxis(5));
        	joyStickLeft.setRumble(RumbleType.kLeftRumble, 1);
        	joyStickLeft.setRumble(RumbleType.kRightRumble, 1);
		}else{
			singleStickArcade();
        	joyStickLeft.setRumble(RumbleType.kLeftRumble, 1);
        	joyStickLeft.setRumble(RumbleType.kRightRumble, 0);
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
		intake = new Victor(6);
		shooterRight = new Spark(0);
		shooterLeft = new Spark(5);
		climberLeft = new Victor(2);
		climberRight = new Victor(7);
		elevator = new Victor(1);
		// agitator = new Victor(6);

		// Encoder inits and instantiations
		shooterRightEnc = new Encoder(0, 1, true, Encoder.EncodingType.k1X);
		shooterRightEnc.setMaxPeriod(1);
		shooterRightEnc.setDistancePerPulse(36);

		shooterRightEnc.setMinRate(10);
		shooterRightEnc.setSamplesToAverage(32);
		
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
		// joyStickRight = new Joystick(1);

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
		logitechStart = new JoystickButton(joyStickLeft, 10);
		logitechBack = new JoystickButton(joyStickLeft, 9);

		// rightTrim = SmartDashboard.getNumber("DB/Slider 3", 1.0);
		// if(rightTrim == 0){ SmartDashboard.putNumber("DB/Slider 3", 1);
		// rightTrim = 1;}

		gearTrayCylinder = new DoubleSolenoid(2, 1);// port 0 failed, changed to
													// 2
		ballBlockCylinder = new DoubleSolenoid(3, 4);
		gearTrayCylinder.set(DoubleSolenoid.Value.kReverse);
		ballBlockCylinder.set(DoubleSolenoid.Value.kForward);

		// compressor port init
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(false);

		// limit switch init
		// imitSwitch = new DigitalInput(1);

		current = power.getCurrent(15);
		System.out.println(current);

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

		rotationPos = 0;
		
		botPosition = SmartDashboard.getString("DB/String 0", "Is Bot at loading, center, or boiler?");
		allianceColor = SmartDashboard.getString("DB/String 1", "Alliance Color red or blue?");
		chooseBoilerOrLoading = SmartDashboard.getString("DB/String 2", "Going boiler or loading side or nil?");
		baselineCross = SmartDashboard.getString("DB/String 3", "Only Cross Baseline? yes or no");
		gearPlacement = SmartDashboard.getString("DB/String 4", "Place gear right, left, middle, or nil?");
		shoot = SmartDashboard.getString("DB/String 5", "Shoot Before Hopper Pickup? yes or no");
		shootAfterHopper = SmartDashboard.getString("DB/String 6", "Shoot After Hopper Pickup? yes or no");
		hopperPickup = SmartDashboard.getString("DB/String 7", "Pickup balls from hopper? yes or no");
		
		//logicError = SmartDashboard.getString("DB/String 6", "No error");
		//endgameAutonomous = SmartDashboard.getString("DB/String 8", "Hah....BASIC");
		
		// NavX instantiation
		try {
			//ahrs = new AHRS(SerialPort.Port.kUSB);
			 ahrs = new AHRS(I2C.Port.kMXP); //WE WILL NEED I2C IN THE FUTURE.
			// RIGHT NOW WE WILL STICK WITH USB
		} catch (RuntimeException ex) {
			System.out.println("NavX instantiation error");
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
	}

	private void operatorControl() {
		if (isOperatorControl() && isEnabled()) {

			Timer.delay(
					0.020); /* wait for one motor update time period (50Hz) */

			/*
			 * boolean zero_yaw_pressed = stick.getTrigger(); if (
			 * zero_yaw_pressed ) { ahrs.zeroYaw(); }
			 */

			/* Display 6-axis Processed Angle Data */
			SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
			SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
			SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
			SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
			SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

			/* Display tilt-corrected, Magnetometer-based heading (requires */
			/* magnetometer calibration to be useful) */

			SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

			/*
			 * Display 9-axis Heading (requires magnetometer calibration to be
			 * useful)
			 */
			SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

			/*
			 * These functions are compatible w/the WPI Gyro Class, providing a
			 * simple
			 */
			/* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

			SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
			SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

			/*
			 * Display Processed Acceleration Data (Linear Acceleration, Motion
			 * Detect)
			 */

			SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
			SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
			SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
			SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

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

			SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
			SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
			SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
			SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());

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

			/* Omnimount Yaw Axis Information */
			/*
			 * For more info, see
			 * http://navx-mxp.kauailabs.com/installation/omnimount
			 */
			AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
			SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
			SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

			/* Sensor Board Information */
			SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());

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
	    
		ballBlockCylinder.set(DoubleSolenoid.Value.kReverse);
		gearTrayCylinder.set(DoubleSolenoid.Value.kReverse);
		
		driveMotors(0, 0);
		
		if((botPosition == "right" || botPosition == "left") &&			
				(allianceColor == "blue" || allianceColor == "red") &&
				(chooseBoilerOrLoading == "boiler" && chooseBoilerOrLoading == "loading") &&
				baselineCross == "yes" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){	
			autoMode = RobotStates.baseline;
		}
		if((botPosition == "middle") &&										
				(allianceColor == "blue" || allianceColor == "red") &&
				chooseBoilerOrLoading == "nil" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){	
			autoMode = RobotStates.gearMiddle;
		}
		
		//**************BOILER-BASED FUNCTIONS**************
		
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearMiddleBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearMiddleBoilerRight;
		}
		if(botPosition == "left" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearBoilerLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearBoilerRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearBoilerRightWhileMiddle;
		}
		if(botPosition == "left" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "yes" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.shootOnlyBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "yes" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.shootOnlyBoilerLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "yes" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.shootOnlyBoilerRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "yes" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.shootOnlyBoilerRightWhileMiddle;
		}
		if(botPosition == "left" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperOnlyBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperOnlyBoilerLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperOnlyBoilerRight;
		}
		if(botPosition == "right" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperOnlyBoilerRightWhileMiddle;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "yes" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearMiddleShootBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "yes" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearMiddleShootBoilerRight;
		}
		if(botPosition == "left" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "yes" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearShootBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "yes" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearShootBoilerLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "yes" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearShootBoilerRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "yes" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearShootBoilerRightWhileMiddle;
		}
		if(botPosition == "left" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperShootBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperShootBoilerLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperShootBoilerRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperShootBoilerRightWhileMiddle;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearMiddleHopperBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearMiddleHopperBoilerRight;
		}
		if(botPosition == "left" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperBoilerLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperBoilerRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperBoilerRightWhileMiddle;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearMiddleHopperShootBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearMiddleHopperShootBoilerRight;
		}
		if(botPosition == "left" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "no" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperShootBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "no" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperShootBoilerLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "no" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperShootBoilerRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "no" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperShootBoilerRightWhileMiddle;
		}
		if(botPosition == "left" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "yes" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.shootHopperShootBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "yes" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.shootHopperShootBoilerLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "yes" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.shootHopperShootBoilerRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "yes" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.shootHopperShootBoilerRightWhileMiddle;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "yes" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.ultimateAutoGearMiddleBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "yes" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.ultimateAutoGearMiddleBoilerRight;
		}
		if(botPosition == "left" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "yes" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.ultimateAutoBoilerLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "yes" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.ultimateAutoBoilerLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "yes" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.ultimateAutoBoilerRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "boiler" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "yes" &&
				shootAfterHopper == "yes" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.ultimateAutoBoilerRightWhileMiddle;
		}
		
		//**************LOADING-BASED FUNCTIONS**************
		
		if(botPosition == "left" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.loadingOnlyLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.loadingOnlyLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.loadingOnlyRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.loadingOnlyRightWhileMiddle;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearMiddleLoadingLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearMiddleLoadingRight;
		}
		if(botPosition == "left" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearLoadingLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearLoadingLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearLoadingRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "no"){
			autoMode = RobotStates.gearLoadingRightWhileMiddle;
		}
		if(botPosition == "left" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperLoadingLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperLoadingLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperLoadingRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "nil" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.hopperLoadingRightWhileMiddle;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearMiddleHopperLoadingLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "middle" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearMiddleHopperLoadingRight;
		}
		if(botPosition == "left" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperLoadingLeft;
		}
		if(botPosition == "middle" &&
				allianceColor == "red" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "left" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperLoadingLeftWhileMiddle;
		}
		if(botPosition == "right" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperLoadingRight;
		}
		if(botPosition == "middle" &&
				allianceColor == "blue" &&
				chooseBoilerOrLoading == "loading" &&
				baselineCross == "no" &&
				gearPlacement == "right" &&
				shoot == "no" &&
				shootAfterHopper == "no" &&
				hopperPickup == "yes"){
			autoMode = RobotStates.gearHopperLoadingRightWhileMiddle;
		}
	}


	public void autonomousPeriodic() {
/*NOTE: the notions of the method driveMotors are to be paired with encoder data.
 * Make sure to pair!!!!!!!
 */
		
		switch(autoMode){
			case baseline: //Cross baseline
				encoderMacro(250);
				if(cycleCounter < 250){
					driveMotors(1, 1);
				}
				break;
			case gearMiddle:
				if(cycleCounter < 100){
					driveMotors(1, 1);
				}else if(cycleCounter < 350){
					driveMotors(0.3, 0.3);
				}else if(cycleCounter < 450){
					gearTrayCylinder.set(DoubleSolenoid.Value.kForward);
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
				shootAutonomous(0);
				break;
			case shootOnlyBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				shootAutonomous(0);
				break;
			case shootOnlyBoilerRight:
				gotoBoilerRightWhileRightPosition();
				shootAutonomous(0);
				break;
			case shootOnlyBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				shootAutonomous(0);
				break;
			case hopperOnlyBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				hopperWhileBoilerLeft();
				break;
			case hopperOnlyBoilerLeftWhileMiddle:
			case hopperOnlyBoilerRight:
			case hopperOnlyBoilerRightWhileMiddle:
			case gearMiddleShootBoilerLeft: //Priority
			case gearMiddleShootBoilerRight: //Priority
			case gearShootBoilerLeft:
			case gearShootBoilerLeftWhileMiddle:
			case gearShootBoilerRight:
			case gearShootBoilerRightWhileMiddle:
			case hopperShootBoilerLeft:
			case hopperShootBoilerLeftWhileMiddle:
			case hopperShootBoilerRight:
			case hopperShootBoilerRightWhileMiddle:
			case gearMiddleHopperBoilerLeft:
			case gearMiddleHopperBoilerRight:
			case gearHopperBoilerLeft:
			case gearHopperBoilerLeftWhileMiddle:
			case gearHopperBoilerRight:
			case gearHopperBoilerRightWhileMiddle:
			case gearMiddleHopperShootBoilerLeft:
			case gearMiddleHopperShootBoilerRight:
			case gearHopperShootBoilerLeft:
			case gearHopperShootBoilerLeftWhileMiddle:
			case gearHopperShootBoilerRight:
			case gearHopperShootBoilerRightWhileMiddle:
			case shootHopperShootBoilerLeft:
			case shootHopperShootBoilerLeftWhileMiddle:
			case shootHopperShootBoilerRight:
			case shootHopperShootBoilerRightWhileMiddle:
			case ultimateAutoGearMiddleBoilerLeft:
			case ultimateAutoGearMiddleBoilerRight:
			case ultimateAutoBoilerLeft:
			case ultimateAutoBoilerLeftWhileMiddle:
			case ultimateAutoBoilerRight:
			case ultimateAutoBoilerRightWhileMiddle:
			case loadingOnlyLeft:
			case loadingOnlyLeftWhileMiddle:
			case loadingOnlyRight:
			case loadingOnlyRightWhileMiddle:
			case gearMiddleLoadingLeft:
			case gearMiddleLoadingRight:
			case gearLoadingLeft:
			case gearLoadingLeftWhileMiddle:
			case gearLoadingRight:
			case gearLoadingRightWhileMiddle:
			case hopperLoadingLeft:
			case hopperLoadingLeftWhileMiddle:
			case hopperLoadingRight:
			case hopperLoadingRightWhileMiddle:
			case gearMiddleHopperLoadingLeft:
			case gearMiddleHopperLoadingRight:
			case gearHopperLoadingLeft:
			case gearHopperLoadingLeftWhileMiddle:
			case gearHopperLoadingRight:
			case gearHopperLoadingRightWhileMiddle:
				break;
		}		
		cycleCounter++;
		
	}

	public void teleopInit() {
		
	}

	public void teleopPeriodic() {

		Scheduler.getInstance().run();
		
		rotationCount = shooterRightEnc.get();
		rotationRate = shooterRightEnc.getRate();
		double distance = shooterRightEnc.getDistance();
		boolean direction = shooterRightEnc.getDirection();
		boolean stopped = shooterRightEnc.getStopped();
		rotationPeriod = shooterRightEnc.getRaw();
		/*System.out.println("************");
		System.out.println(distance);
		System.out.println(direction);
		System.out.println(stopped);
		System.out.println(rotationCount);
		System.out.println(rotationRate);
		System.out.println(rotationPeriod);
		System.out.println("************");*/
		
		//System.out.println(shooterEncoder.getDistance());
		//System.out.println(shooterEncoder.get());


		switchDriveModes();

		testForCorrectionMode();

		toggleShooterMotor();

		toggleIntake();
		
		//checkClimberState();
		
		toggleElevator();

		
		toggleGearTray();
		
		operatorControl();

	}

	public void testPeriodic() {
		LiveWindow.run();
		operatorControl();
	}
}
