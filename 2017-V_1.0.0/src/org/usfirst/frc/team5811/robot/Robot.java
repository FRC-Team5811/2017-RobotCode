package org.usfirst.frc.team5811.robot;

import org.usfirst.frc.team5811.robot.commands.*;
import org.usfirst.frc.team5811.robot.subsystems.*;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public static Controls oi;
	public static RobotMap map;
	
	public static Climber climber;
	public static PowerManagement power;
	public static CompressorSubsystem compressor;
	public static Elevator elevator;
	public static Shooter shooter;
	
	Command autonomousCommand;
	SendableChooser chooser = new SendableChooser();
	
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
	
	boolean shouldBeRunningIntake;
	boolean wasPressedLeftBumper;
	boolean wasPressedLogitechA;
	boolean shouldBeRunningClimberDown;
	boolean wasPressedLogitechY;
	
	
	boolean shouldBeRunningGearTray;
	boolean wasPressedLogitechB;
	
	boolean shouldBeRunningShifter;
	boolean wasPressedRightStick;

	boolean shouldBeRunningCorrect;
	boolean wasPressedBackButton;
	
    boolean shouldBeRunningAutoTurn;
    boolean wasPressedStart;

    //used in "correction" mode
	float rotationPos;
	
	private void dualStick(){
		arcadeDrive(-Controls.driverJoystick.getRawAxis(1),Controls.driverJoystick.getRawAxis(2));
	}
	private void slowMove(double reduction){
		RobotMap.shifterCylinder.set(DoubleSolenoid.Value.kForward);
		arcadeDrive((-Controls.driverJoystick.getRawAxis(1)*reduction), (Controls.driverJoystick.getRawAxis(2)*reduction));
	}
	
	public void intakeOnOff(double speed){
		RobotMap.intakeMotor.set(speed);
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
		if (Controls.driverBack.get()) {
			if (!wasPressedBackButton) {
				rotationPos = (float) RobotMap.ahrs.getAngle();
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
		climber.set(Controls.manipulatorJoystick.getY());
	} 
	
	private void toggleResExpansion(){
		if (Controls.manipulatorY.get()) {
			if (!wasPressedLogitechB) {
				shouldBeRunningGearTray = !shouldBeRunningGearTray;
			}
			wasPressedLogitechB = true;
		} else {
			wasPressedLogitechB = false;
		}

		if (shouldBeRunningGearTray) {
			RobotMap.reservoirCylinder.set(DoubleSolenoid.Value.kReverse);
		} else {
			RobotMap.reservoirCylinder.set(DoubleSolenoid.Value.kForward);
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
		if(Controls.driverRightBumper.get()){
			shifterDelay();
			RobotMap.shifterCylinder.set(DoubleSolenoid.Value.kForward);
		}
		if(Controls.driverLeftBumper.get()){
			shifterDelay();
			RobotMap.shifterCylinder.set(DoubleSolenoid.Value.kReverse);
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
		RobotMap.frontLeftDriveMotor.set(-speedLeftDM);
		RobotMap.backLeftDriveMotor.set(-speedLeftDM);
		RobotMap.frontRightDriveMotor.set(speedRightDM);
		RobotMap.backRightDriveMotor.set(speedRightDM);
	}

	// 2 STICK DRIVE METHOD
	private void arcadeDrive(double throttle, double turn) {
		driveMotors((throttle + turn), (throttle - turn));
	}
	
    private void turnMacro(float degrees){
    	RobotMap.ahrs.reset();
    	float nowRot = (float) RobotMap.ahrs.getAngle();
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
    		nowRot = (float) RobotMap.ahrs.getAngle();
    	}
    }
    
    private void driveStraightFeet(float feet){
    	RobotMap.ahrs.reset();
    	float nowRot = (float) RobotMap.ahrs.getAngle();
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
        		nowRot = (float) RobotMap.ahrs.getAngle();
        	}
    	}
    	else{
    		driveMotors(0,0);
    	}
    }

	// CORRECTION METHOD. WE USE THE VALUE QUARTERNION Z FOR ROTATIONAL
	// POSTITIONING
    //Spectre says HI 
	private void correct() {
		float nowRot = (float) RobotMap.ahrs.getAngle();
		System.out.println(rotationPos);
		System.out.println(nowRot);
		if (nowRot >= rotationPos + 10) {
			driveMotors(-.5,-.5);
		}
		if (nowRot <= rotationPos - 10) {
			driveMotors(.5,.5);
		}
    }


    public void robotInit() {
    	//this should be first as nothing can exist without it
    	map = new RobotMap();
    	    	
    	compressor = new CompressorSubsystem(new Compressor(RobotMap.CompressorChannel));
    	power = new PowerManagement(new PowerDistributionPanel());
    	climber = new Climber();
    	elevator = new Elevator();
    	shooter = new Shooter();

		chooser = new SendableChooser();
		// chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		chooser.addObject("My Auto", "My Auto");
		SmartDashboard.putData("Auto mode", chooser);
		
		System.out.println(SmartDashboard.getBoolean("DB/Button 0", false));
		
		autoMode = RobotStates.noStringNoMove;

		
		// Boolean Toggle Switch
		shouldBeRunningSwitch = false;
		wasPressedLeftStick = false;

		shouldBeRunningIntake = false;
		wasPressedLeftBumper = false;
		wasPressedLogitechA = false;
		shouldBeRunningClimberDown = false;
		wasPressedLogitechY = false;
		shouldBeRunningGearTray = false;
		wasPressedLogitechB = false;
		
		spinUpComplete = false;

		wasPressedBackButton = false;
		shouldBeRunningCorrect = false;
		
		shouldBeRunningShifter = false;
		wasPressedRightStick = false;

		rotationPos = 0;
		
		driveState = true;
		
		//this should go last after all the subsystems have be initialized
		oi = new Controls();
		
		
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

			SmartDashboard.putNumber("IMU_TotalYaw", RobotMap.ahrs.getAngle());

			/* Connectivity Debugging Support */
			SmartDashboard.putNumber("IMU_Byte_Count", RobotMap.ahrs.getByteCount());
			SmartDashboard.putNumber("IMU_Update_Count", RobotMap.ahrs.getUpdateCount());
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
		
		
		RobotMap.shifterCylinder.set(DoubleSolenoid.Value.kReverse);
		RobotMap.reservoirCylinder.set(DoubleSolenoid.Value.kReverse);
		
		cycleCounter = 0;
		driveMotors(0, 0);
		
	}

	
	public void autonomousPeriodic() {
		
		Scheduler.getInstance().run();
		 
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
		
		distance = RobotMap.driveEncoder.getDistance();
		
		rotationCountForDrive = RobotMap.driveEncoder.get();
		rotationRateForDrive = RobotMap.driveEncoder.getRate();
		
		System.out.println("************");
		System.out.println(distance);
		System.out.println(rotationCountForDrive);
		System.out.println(rotationRateForDrive);
		System.out.println("************");
		System.out.println("Angle: "+ RobotMap.ahrs.getAngle());
		System.out.println("Elevator Current: "+power.elevator());
		System.out.println("Intake Current: "+power.intake());
		System.out.println("Front Right Drive Current: "+ power.frontRightDrive());
		System.out.println("Back Right Drive Current: "+ power.backLeftDrive());
		System.out.println("Front Left Drive Current: "+ power.frontLeftDrive());
		System.out.println("Back Left Drive Current: "+ power.backLeftDrive());
		System.out.println("Climber 1 Current: "+ power.climber1());
		System.out.println("Climber 2 Current: "+ power.climber2());
		
		
		if(Controls.manipulatorA.get()){
			intakeOnOff(.5);         //INTAKE CODE 
			shouldBeRunningIntake = true;
		}
		else if(Controls.manipulatorB.get()){
			intakeOnOff(0);
			shouldBeRunningIntake = false;
		}
		
		
		if(Controls.driverX.get()){
			driveState = true;
		}
		if(Controls.driverY.get()){
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
		
		//switchDriveModes();  

		testForCorrectionMode();

		//toggleShooterMotor();

		//toggleIntake();
		
		checkClimberState();

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
