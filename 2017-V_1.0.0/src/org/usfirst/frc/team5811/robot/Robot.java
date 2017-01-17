package org.usfirst.frc.team5811.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.usfirst.frc.team5811.robot.commands.ExampleCommand;
import org.usfirst.frc.team5811.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.buttons.JoystickButton; 

public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

    Command autonomousCommand;
    SendableChooser chooser = new SendableChooser();

    Joystick joyStickLeft;
    Joystick joyStickRight;
    Joystick logitech;
    
    JoystickButton logitechY;
    JoystickButton logitechA;
    JoystickButton logitechX;
    JoystickButton logitechB;
    JoystickButton logitechLeftBumper;
    JoystickButton logitechRightBumper;

    int direction; 
    boolean stupidIntake;
    
    //Motors
    Victor frontLeftDriveMotor;
    Victor frontRightDriveMotor;
    Victor backLeftDriveMotor;
    Victor backRightDriveMotor;
    Victor intake;
    Victor shooter;
    Victor climber;
    Victor agitator;
    Victor elevator;
    
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
    
    double leftTrim;
    double rightTrim;
    
    //Global Speed Values
    double leftSpeed;
    double rightSpeed;
    
    double throttleGain;
    double turningGain;
    
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
        frontLeftDriveMotor.set(joyStickLeft.getY()+joyStickLeft.getX());
        frontRightDriveMotor.set(joyStickLeft.getY()-joyStickLeft.getX());
        backLeftDriveMotor.set(joyStickLeft.getY()+joyStickLeft.getX());
        backRightDriveMotor.set(joyStickLeft.getY()-joyStickLeft.getY());
        
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
       frontLeftDriveMotor = new Victor(9);
       frontRightDriveMotor = new Victor(1);
       backLeftDriveMotor = new Victor(8); 
       backRightDriveMotor = new Victor(0);
       
       //Accessory motors
       intake = new Victor(4);
       shooter = new Victor(5);
       climber = new Victor(3);
       elevator = new Victor(7);
       agitator = new Victor(6);
       
       joyStickLeft = new Joystick(0);
       joyStickRight = new Joystick(1);
       logitech = new Joystick(2);
      
       logitechY= new JoystickButton(logitech, 4);
       logitechA= new JoystickButton(logitech, 1);
       logitechX= new JoystickButton(logitech, 3);
       logitechB= new JoystickButton(logitech, 2);
       logitechLeftBumper= new JoystickButton(logitech, 5);
       logitechRightBumper= new JoystickButton(logitech, 6);
       stupidIntake = false;
       
       rightTrim = SmartDashboard.getNumber("DB/Slider 3", 1.0);
       if(rightTrim == 0){ SmartDashboard.putNumber("DB/Slider 3", 1); rightTrim = 1;}
       
       gearTrayCylinder = new DoubleSolenoid(2,1);//port 0 failed, changed to 2
       ballBlockCylinder = new DoubleSolenoid(3,4);
       gearTrayCylinder.set(DoubleSolenoid.Value.kReverse);
       ballBlockCylinder.set(DoubleSolenoid.Value.kForward);
       
       //set cycle counter
       cycleCounter = 0;
          
       //compressor port init
       compressor = new Compressor(0);
       compressor.setClosedLoopControl(false);
       
       throttleGain = 1;
       turningGain = .8;
     
       //limit switch init
       limitSwitch =  new DigitalInput(1);
       
       current = power.getCurrent(15);
       System.out.println(current);
       
       //Boolean State Change instantiation
       shooterStateChange = true;
       intakeStateChange = true;
       elevatorStateChange = true;
       climberAscendStateChange = true;
       climberDescendStateChange = true;
       gearTrayStateChange = true;
       
    
    }
   
    private void operatorControl(){
    	//call if limit switch is used
    	/*while(limitSwitch.get()) {
    		System.out.println("limit switch");
    	}*/
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
        
        //agitator
        agitator.set(.75);
        //shooter
        if(logitechRightBumper.get() && shooterStateChange == true) {
        	shooter.set(.75);
        	shooterStateChange = false;
        	//if(encoderValue=100%){
        	//ballBlockCylinder.set(DoubleSolenoid.Value.kReverse);
        	//}
        }	
      
        if(logitechRightBumper.get() && shooterStateChange == false){
    		shooter.set(0);
    		shooterStateChange = true;
    		ballBlockCylinder.set(DoubleSolenoid.Value.kForward);
    	}
        //intake
    	if(logitechLeftBumper.get() && intakeStateChange == true){
    		intake.set(1);
    		SmartDashboard.putNumber("INTAKE ON", 101);
    		intakeStateChange = false;
    	}
 
    	if(logitechLeftBumper.get() && intakeStateChange == false){
    		intake.set(0);
    		SmartDashboard.putNumber("INTAKE OFF", 101);
    		intakeStateChange = true; 
    	}
    	//climber down
    	if(logitechA.get() && climberDescendStateChange == true){
    		climber.set(-1);
    		climberDescendStateChange = false;
    	}
    	if(logitechA.get() && climberDescendStateChange == false){
    		climber.set(0);
    		climberDescendStateChange = true;
    	}
    	//climber up
    	if(logitechY.get() && climberAscendStateChange == true){
    		climber.set(1);
    		agitator.set(0);
    		climberAscendStateChange = false;
    	}
    	if(logitechY.get() && climberAscendStateChange == false){
    		climber.set(0);
    		climberAscendStateChange = true;
    	}
    	/*
    	if(current > n){
    		climber.set(0);
    	}
    	*/
    	//elevator
    	if(logitechX.get() && elevatorStateChange == true){
    		elevator.set(1);
    		SmartDashboard.putNumber("ELEVATOR ON", 101);
    		elevatorStateChange = false;
    	}
    	if(logitechX.get() && elevatorStateChange == false){
    		elevator.set(0);
    		SmartDashboard.putNumber("ELEVATOR OFF", 101);
    		elevatorStateChange = true;
    	}
    	/*
    	if(current > n){
    		elevator.set(0);
    		elevatorStateChange = true;
    	}
    	*/
    	//geartray
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
    }
    public void testPeriodic() {
        LiveWindow.run();  
    }
}

