package org.usfirst.frc.team5811.robot;

import org.usfirst.frc.team5811.robot.subsystems.*;
import org.usfirst.frc.team5811.robot.subsystems.DriveTrain.DriveMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public static Controls oi;
	public static RobotMap map;
	
	//Subsystems declarations
	public static Climber climber;
	public static PowerManagement power;
	public static CompressorSubsystem compressor;
	public static Elevator elevator;
	public static Shooter shooter;
	public static Intake intake;
	public static Wings wings;
	public static DriveTrain train;
	public static Shifter shifter;
	
	//stuff
	Command autonomousCommand;
	SendableChooser chooser = new SendableChooser();
	
	//PIDControlled
	public static NavXRotationalController turnController;
	
	//UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

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

    public void robotInit() {
    	//this should be first as nothing can exist without it
    	map = new RobotMap();
    	    	
    	compressor = new CompressorSubsystem(new Compressor(RobotMap.CompressorChannel));
    	power = new PowerManagement(new PowerDistributionPanel());
    	climber = new Climber();
    	elevator = new Elevator();
    	shooter = new Shooter();
    	intake = new Intake();
    	wings = new Wings();
    	train = new DriveTrain();
    	shifter = new Shifter();

		chooser = new SendableChooser();
		// chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		chooser.addObject("My Auto", "My Auto");
		
		SmartDashboard.putData("Auto mode", chooser);
		
		//System.out.println(SmartDashboard.getBoolean("DB/Button 0", false));
		
		autoMode = RobotStates.noStringNoMove;
		
		//this should go last after all the subsystems have be initialized
		oi = new Controls();
		
		turnController = new NavXRotationalController(RobotMap.navx,train);
		
		/*
		SmartDashboard.putString("DB/String 0", "left, right, middle");
		SmartDashboard.putString("DB/String 1", "red, blue");
		SmartDashboard.putString("DB/String 2", "boiler, loading");
		SmartDashboard.putString("DB/String 3", "Cross baseline?");
		SmartDashboard.putString("DB/String 4", "Where to place gear?");
		SmartDashboard.putString("DB/String 5", "Shoot before hopper?");
		SmartDashboard.putString("DB/String 6", "Shoot after hopper?");
		SmartDashboard.putString("DB/String 7", "Pickup at Hopper?");
		*/
		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
		
	}


	public void disabledInit() {
		
	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	public void autonomousInit() {
	    autonomousCommand = (Command) chooser.getSelected();
	    
		/*botPosition = SmartDashboard.getString("DB/String 0", "Is Bot at loading, middle, or boiler?");
		allianceColor = SmartDashboard.getString("DB/String 1", "Alliance Color red or blue?");
		chooseBoilerOrLoading = SmartDashboard.getString("DB/String 2", "Going boiler or loading side or nil?");
		baselineCross = SmartDashboard.getString("DB/String 3", "Only Cross Baseline? yes or no");
		gearPlacement = SmartDashboard.getString("DB/String 4", "Place gear right, left, middle, or nil?");
		shoot = SmartDashboard.getString("DB/String 5", "Shoot Before Hopper Pickup? yes or no");
		shootAfterHopper = SmartDashboard.getString("DB/String 6", "Shoot After Hopper Pickup? yes or no");
		hopperPickup = SmartDashboard.getString("DB/String 7", "Pickup balls from hopper? yes or no");
		*/
		
		RobotMap.shifterCylinder.set(DoubleSolenoid.Value.kReverse);
		RobotMap.reservoirCylinder.set(DoubleSolenoid.Value.kReverse);
		
	}
	
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		
	}

	public void teleopInit() {
		train.transitionToMode(DriveMode.normal);
	}

	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		cycleCounter++;
		
		if(cycleCounter % 100 == 0){
			log("************");
			System.out.println(RobotMap.driveEncoder.getDistance());
			//System.out.println(RobotMap.driveEncoder.get());
			//System.out.println(RobotMap.driveEncoder.getRate());
			log("Angle: "+ RobotMap.navx.getAngle());
			log("Mode: "+ train.getCurrentMode());
			log("************");
			
			//log("Elevator Current: "+power.elevator());
			//log("Intake Current: "+power.intake());
			//log("Front Right Drive Current: "+ power.frontRightDrive());
			//log("Back Right Drive Current: "+ power.backLeftDrive());
			//log("Front Left Drive Current: "+ power.frontLeftDrive());
			//log("Back Left Drive Current: "+ power.backLeftDrive());
			//log("Climber 1 Current: "+ power.climber1());
			//log("Climber 2 Current: "+ power.climber2());
		}
	}

	public void testPeriodic() {
		LiveWindow.run();
		dashboardDisplay();
	}

	private void dashboardDisplay() {
		SmartDashboard.putNumber("IMU_TotalYaw", RobotMap.navx.getAngle());
		SmartDashboard.putNumber("IMU_Byte_Count", RobotMap.navx.getByteCount());
		SmartDashboard.putNumber("IMU_Update_Count", RobotMap.navx.getUpdateCount());
	}
	
	public static void log(String str){
		System.out.println(str);
	}
	
}
