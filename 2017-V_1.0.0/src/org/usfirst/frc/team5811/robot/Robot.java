package org.usfirst.frc.team5811.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.usfirst.frc.team5811.robot.commands.ExampleCommand;
import org.usfirst.frc.team5811.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

    Command autonomousCommand;
    SendableChooser chooser = new SendableChooser();

    Joystick joyStickLeft;
    //Joystick joyStickRight;
    Joystick logitech;
    
    JoystickButton logitechY;
    JoystickButton logitechA;
    JoystickButton logitechX;
    JoystickButton logitechB;
    JoystickButton logitechLeftBumper;
    JoystickButton logitechRightBumper;
    JoystickButton logitechLeftStickPress;
    JoystickButton logitechStart;
    
    //Motors
    Victor frontLeftDriveMotor;
    Victor frontRightDriveMotor;
    Victor backLeftDriveMotor;
    Victor backRightDriveMotor;
    Victor intake;
    Spark shooterRight;;
    Spark shooterLeft;
    Victor climber;
    Victor agitator;
    Victor elevator;
    
    //Encoder definitions and variables
    Encoder shooterEnc;
    int rotationCount;
    double rotationRate;
    boolean encDirection;
    boolean encIfStopped;
    
    double intakePower; 
    
    double autoSelecter;
    boolean releaseToggle;
    boolean raiseAfterRelease;
    boolean returnAfter;
    double turnTime;
    double turnPower;
    boolean shootBall;
    
    int cycleCounter;
    int intakeCounter;
    boolean firstSpikeStarted;
    boolean firstSpikeFinished;
    boolean secondSpikeStarted;
    int secondIntakeCounter;
    double ultrasonicVoltage;
    int rightThreshold;
    int centerThreshold;
    int leftThreshold;
    
    //Boolean state changes
    boolean shooterStateChange;
    boolean intakeStateChange;
    boolean elevatorStateChange;
    boolean climberAscendStateChange;
    boolean climberDescendStateChange;
    boolean gearTrayStateChange;
    boolean leftStickPressStateChange;
    
    //Global Speed Values
    double leftSpeed;
    double rightSpeed;
    
    
    double current;
    double n;
    
    //A cylinder
    DoubleSolenoid gearTrayCylinder;
    DoubleSolenoid ballBlockCylinder;
    
    //COMPRESSOR!!!
    Compressor compressor;
    
    //Limit Switch
    DigitalInput limitSwitch;
       
    //Button boolean
    boolean state;
    boolean previousState;
    
    //power distribution panel
    PowerDistributionPanel power = new PowerDistributionPanel();
    
    //For the pneumatic
    double dpad;
    double logitechDPad;
    
    
  
    private void singleStickArcade(){
        frontLeftDriveMotor.set(joyStickLeft.getX()-joyStickLeft.getY());
        frontRightDriveMotor.set(joyStickLeft.getX()+joyStickLeft.getY());
        backLeftDriveMotor.set(joyStickLeft.getX()-joyStickLeft.getY());
        backRightDriveMotor.set(joyStickLeft.getX()+joyStickLeft.getY());
    }
    
    private void driveMotors(double speedLeftDM, double speedRightDM) {
    	//System.out.println("Command: " + speedLeftDM);
    	frontLeftDriveMotor.set(speedLeftDM);
    	frontRightDriveMotor.set(speedRightDM);
    	backLeftDriveMotor.set(speedLeftDM);
    	backRightDriveMotor.set(speedRightDM);
    }
    
    
    private void arcadeDrive(double throttle, double turn){
    	driveMotors((throttle + turn), (throttle - turn));
    }
    
    
    public void robotInit() {
 
		oi = new OI();
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", new ExampleCommand());
        //chooser.addObject("My Auto", new MyAutoCommand());
        chooser.addObject("My Auto", "My Auto");
        SmartDashboard.putData("Auto mode", chooser);
        System.out.println(SmartDashboard.getBoolean("DB/Button 0", false));
  
        //Motor port instantiating
       frontLeftDriveMotor = new Victor(0);
       frontRightDriveMotor = new Victor(8);
       backLeftDriveMotor = new Victor(1); 
       backRightDriveMotor = new Victor(9);
       
       //Accessory motors
       //intake = new Victor(1);
       shooterRight = new Spark(5);
       shooterLeft = new Spark(4);
       climber = new Victor(3);
       elevator = new Victor(7);
       agitator = new Victor(6);
       
       //Encoder inits and instantiations
       shooterEnc = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
       shooterEnc.setMaxPeriod(.1);
       shooterEnc.setMinRate(10);
       shooterEnc.setDistancePerPulse(5);
       shooterEnc.setReverseDirection(true);
       shooterEnc.setSamplesToAverage(7);
       
       rotationCount = shooterEnc.get();
       rotationRate = shooterEnc.getRate();
       encIfStopped = shooterEnc.getStopped();
       encDirection = shooterEnc.getDirection();//since it is a boolean its either 0 or 1 (obv)...not sure which value is which direction though
      
       joyStickLeft = new Joystick(0);
       //joyStickRight = new Joystick(1);
      
       logitechY= new JoystickButton(joyStickLeft, 4);
       logitechA= new JoystickButton(joyStickLeft, 1);
       logitechX= new JoystickButton(joyStickLeft, 3);
       logitechB= new JoystickButton(joyStickLeft, 2);
       logitechLeftBumper= new JoystickButton(joyStickLeft, 5);
       logitechRightBumper= new JoystickButton(joyStickLeft, 6);
       logitechLeftStickPress = new JoystickButton(joyStickLeft, 9);
       logitechStart = new JoystickButton(joyStickLeft, 7);
       
       //rightTrim = SmartDashboard.getNumber("DB/Slider 3", 1.0);
       //if(rightTrim == 0){ SmartDashboard.putNumber("DB/Slider 3", 1); rightTrim = 1;}
       
       gearTrayCylinder = new DoubleSolenoid(2,1);//port 0 failed, changed to 2
       ballBlockCylinder = new DoubleSolenoid(3,4);
       gearTrayCylinder.set(DoubleSolenoid.Value.kReverse);
       ballBlockCylinder.set(DoubleSolenoid.Value.kForward);
       
       //set cycle counter
       cycleCounter = 0;
          
       //compressor port init
       compressor = new Compressor(0);
       compressor.setClosedLoopControl(false);
       
       //limit switch init
       //imitSwitch =  new DigitalInput(1);
       
      // current = power.getCurrent(15);
       System.out.println(current);
       
       //Boolean State Change instantiation
       shooterStateChange = true;
       intakeStateChange = true;
       elevatorStateChange = true;
       climberAscendStateChange = true;
       climberDescendStateChange = true;
       gearTrayStateChange = true;
       leftStickPressStateChange = true;
    }
   
    private void operatorControl(){
   
    }
    
    public void disabledInit(){

    }
	
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	
    public void autonomousInit() {

       
    }
    public void autonomousPeriodic() {

    }

    public void teleopInit() {

   
    }
    	
    public void teleopPeriodic() {

        Scheduler.getInstance().run();
        singleStickArcade();
        /*
        if (leftStickPressStateChange){
        	arcadeDrive(logitech.getRawAxis(0)+(logitech.getRawAxis(3)-logitech.getRawAxis(2)),logitech.getRawAxis(5));
        	logitech.setRumble(RumbleType.kLeftRumble, 1);
        	logitech.setRumble(RumbleType.kRightRumble, 1);
        	if(logitechLeftStickPress.get()){
            	leftStickPressStateChange = false;
        	}
        }else{
        	singleStickArcade();
        	logitech.setRumble(RumbleType.kLeftRumble, 1);
        	logitech.setRumble(RumbleType.kRightRumble, 0);
        	if(logitechLeftStickPress.get()){
        		leftStickPressStateChange = true;
        	}
        }
        */
        //agitator
        agitator.set(.75);
        if(logitechStart.get()){
        	if(rotationCount < 100){ //might have to be changed
        		shooterRight.set(.75);
        		shooterLeft.set(.75);
        		ballBlockCylinder.set(DoubleSolenoid.Value.kForward);
        		if(rotationCount >= 100){
        			SmartDashboard.putNumber("SHOOTER READY", 101);
        		}
        }
        //shooter and block pneumatic
        /*	
        if(logitechRightBumper.get()){
        	if(shooterStateChange){
        		shooterRight.set(.75);
        		shooterLeft.set(.75);
        		ballBlockCylinder.set(DoubleSolenoid.Value.kReverse);
            	shooterStateChange = false;
            	
        	}else{
        		shooterRight.set(0);
        		shooterLeft.set(0);
        		shooterStateChange = true;
        		ballBlockCylinder.set(DoubleSolenoid.Value.kForward);
        	}
        }
        */
       
        if(logitechY.get() == true){
        	shooterLeft.set(-.75);
        	shooterRight.set(-.75);
        }
        if(logitechA.get()==true){
        	shooterLeft.set(0);
        	shooterRight.set(0);
        }
        
        System.out.println(shooterLeft.get());
        //intake
        if(logitechLeftBumper.get()){
        	if(intakeStateChange){
        		intake.set(1);
        		SmartDashboard.putNumber("INTAKE ON", 101);
        		intakeStateChange = false;
        	}else{
        		intake.set(0);
        		SmartDashboard.putNumber("INTAKE OFF", 101);
        		intakeStateChange = true; 
        	}
        }
    	/*if(logitechLeftBumper.get() && intakeStateChange == true){
    		intake.set(1);
    		SmartDashboard.putNumber("INTAKE ON", 101);
    		intakeStateChange = false;
    	}
 
    	if(logitechLeftBumper.get() && intakeStateChange == false){
    		intake.set(0);
    		SmartDashboard.putNumber("INTAKE OFF", 101);
    		intakeStateChange = true; 
    	}*/
    	
    	//climber down
        if(logitechA.get()){
        	if(climberDescendStateChange){
        		climber.set(-1);
        		climberDescendStateChange = false;
        	}else{
        		climber.set(0);
        		climberDescendStateChange = true;
        	}
        }
    	/*if(logitechA.get() && climberDescendStateChange == true){
    		climber.set(-1);
    		climberDescendStateChange = false;
    	}
    	if(logitechA.get() && climberDescendStateChange == false){
    		climber.set(0);
    		climberDescendStateChange = true;
    	}*/
        
    	//climber up
        if(logitechY.get()){
        	if(climberAscendStateChange){
        		climber.set(1);
        		agitator.set(0);
        		climberAscendStateChange = false;
        	}else{
        		climber.set(0);
        		climberAscendStateChange = true;
        	}
        }
    	/*if(logitechY.get() && climberAscendStateChange == true){
    		climber.set(1);
    		agitator.set(0);
    		climberAscendStateChange = false;
    	}
    	if(logitechY.get() && climberAscendStateChange == false){
    		climber.set(0);
    		climberAscendStateChange = true;
    	}*/
    	
    	/*
    	if(current > n){
    		climber.set(0);
    	}
    	*/
    	
    	//elevator
        if(logitechX.get()){
        	if(elevatorStateChange){
        		elevator.set(1);
        		SmartDashboard.putNumber("ELEVATOR ON", 101);
        		elevatorStateChange = false;
        	}else{
        		elevator.set(0);
        		SmartDashboard.putNumber("ELEVATOR OFF", 101);
        		elevatorStateChange = true;
        	}
        }
    	/*if(logitechX.get() && elevatorStateChange == true){
    		elevator.set(1);
    		SmartDashboard.putNumber("ELEVATOR ON", 101);
    		elevatorStateChange = false;
    	}
    	if(logitechX.get() && elevatorStateChange == false){
    		elevator.set(0);
    		SmartDashboard.putNumber("ELEVATOR OFF", 101);
    		elevatorStateChange = true;
    	}*/
    	
    	//gear tray
        if(logitechB.get()){
        	if(gearTrayStateChange){
        		gearTrayCylinder.set(DoubleSolenoid.Value.kForward);
       		 SmartDashboard.putNumber("GEAR TRAY OUT", 101);
       		 gearTrayStateChange = false;
        	}else{
        		gearTrayCylinder.set(DoubleSolenoid.Value.kReverse);
       		 SmartDashboard.putNumber("GEAR TRAY IN", 101);
       		 gearTrayStateChange = true;
        	}
        }
        }
    	/*
    	 if(logitechB.get() && gearTrayStateChange == true){
    		 gearTrayCylinder.set(DoubleSolenoid.Value.kForward);
    		 SmartDashboard.putNumber("GEAR TRAY OUT", 101);
    		 gearTrayStateChange = false;
    	}
    	if(logitechB.get() && gearTrayStateChange == false){
    		 gearTrayCylinder.set(DoubleSolenoid.Value.kReverse);
    		 SmartDashboard.putNumber("GEAR TRAY IN", 101);
    		 gearTrayStateChange = true;
    	}
    	*/
    }
    public void testPeriodic() {
        LiveWindow.run();  
    }
}
