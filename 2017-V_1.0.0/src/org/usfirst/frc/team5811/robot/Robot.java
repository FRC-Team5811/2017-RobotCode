package org.usfirst.frc.team5811.robot;

import org.usfirst.frc.team5811.robot.commands.*;
import org.usfirst.frc.team5811.robot.subsystems.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public static Controls oi = new Controls();
	public static Climber climber;
	public static PowerManagement power;
	public static CompressorSubsystem compressor;
	
	
	Command autonomousCommand;
	SendableChooser chooser = new SendableChooser();

	// NavX
	AHRS ahrs;

	// Motors
	Victor frontLeftDriveMotor;
	Victor frontRightDriveMotor;
	Victor backLeftDriveMotor;
	Victor backRightDriveMotor;
	
	Victor intake;
	
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


	// A cylinder
	DoubleSolenoid shifterCylinder;
	DoubleSolenoid reservoirCylinder;
	
	float rotationPos;
	float macroPos;
	
	private void dualStick(){
		arcadeDrive(-Controls.joyStickLeft.getRawAxis(1),Controls.joyStickLeft.getRawAxis(2));
	}
	private void slowMove(double reduction){
		shifterCylinder.set(DoubleSolenoid.Value.kForward);
		arcadeDrive((-Controls.joyStickLeft.getRawAxis(1)*reduction), (Controls.joyStickLeft.getRawAxis(2)*reduction));
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
		if (Controls.logitechBack.get()) {
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
		climber.set(Controls.joyStickRight.getY());
	} 
	
	private void toggleElevator() {
		
		// ELEVATOR
		if (Controls.logitechX2.get()) {
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
		if (Controls.logitechY2.get()) {
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
		if(Controls.logitechRightBumper.get()){
			shifterDelay();
			shifterCylinder.set(DoubleSolenoid.Value.kForward);
		}
		if(Controls.logitechLeftBumper.get()){
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


    public void robotInit() {
    	
    	compressor = new CompressorSubsystem(new Compressor(Map.CompressorChannel));
    	power = new PowerManagement(new PowerDistributionPanel());
    	
    	climber = new Climber(
    		new Victor(Map.leftClimberMotor), 
    		new Victor(Map.rightClimberMotor)
    	);

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
		
		if(Controls.logitechA2.get()){
			intakeOnOff(.5);         //INTAKE CODE 
			shouldBeRunningIntake = true;
		}
		else if(Controls.logitechB2.get()){
			intakeOnOff(0);
			shouldBeRunningIntake = false;
		}
		
		
		if(Controls.logitechX.get()){
			driveState = true;
		}
		if(Controls.logitechY.get()){
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

		intake = power.elevator();
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
