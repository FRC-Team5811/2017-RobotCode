package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.Controls;
import org.usfirst.frc.team5811.robot.Robot;
import org.usfirst.frc.team5811.robot.RobotMap;
import org.usfirst.frc.team5811.robot.commands.RunDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {
	

    //used in "correction" mode
	float rotationPos;
	double distance;
	
	//this should be in RobotMap - however it seemed to be failing when using static reference
	public AHRS navx;
	RobotDrive drive = RobotMap.driveTrain;
	DoubleSolenoid shifter = RobotMap.shifterCylinder;
	Encoder encoder = RobotMap.driveEncoder;
	

	public DriveTrain() {
		// NavX instantiation
		navx = new AHRS(SerialPort.Port.kUSB);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		setDefaultCommand(new RunDrive());
	}
	
	public void runDrive(){
		
		double left = -Controls.driverJoystick.getRawAxis(1);
		double right = -Controls.driverJoystick.getRawAxis(2);
		
		move(left,right);
		
		distance = encoder.getDistance();
	}
	
	public void shifterDelay(){
		int cycleCounterTele = 0;
		while(cycleCounterTele < 20){
			//driveMotors(.3,-.3);
			move(.3,-.3);
			cycleCounterTele++;
		}
	}
	
	private void move(double forwards, double rotation){
		drive.arcadeDrive(forwards, rotation);
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
			shifter.set(DoubleSolenoid.Value.kForward);
		}
		if(Controls.driverLeftBumper.get()){
			shifterDelay();
			shifter.set(DoubleSolenoid.Value.kReverse);
		}
	}
	
    private void turnMacro(float degrees){
    	navx.reset();
    	float nowRot = (float) navx.getAngle();
    	double outputDirection;
    	double outputPower;
    	while(degrees+5 > nowRot && nowRot > degrees-5){
    		if(nowRot > degrees){
    			outputDirection = 1;
    		}else{
    			outputDirection = -1;
    		}
    		outputPower = outputDirection*(((nowRot-degrees)/200)+.1);
    		//driveMotors(outputPower,-outputPower);
    		nowRot = (float) navx.getAngle();
    	}
    }
    
    private void driveStraightFeet(float feet){
    	navx.reset();
    	float nowRot = (float) navx.getAngle();
    	double outputDirection;
    	double outputPower;
    	//double currentLocation;
    	/*if(distance < feet){
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
    	}*/
    }

	// CORRECTION METHOD. WE USE THE VALUE QUARTERNION Z FOR ROTATIONAL
	// POSTITIONING
    //Spectre says HI 
	private void correct() {
		float nowRot = (float) navx.getAngle();
		System.out.println(rotationPos);
		System.out.println(nowRot);
		if (nowRot >= rotationPos + 10) {
			//driveMotors(-.5,-.5);
		}
		if (nowRot <= rotationPos - 10) {
			//driveMotors(.5,.5);
		}
    }
	
	private void slowMove(double reduction){
		RobotMap.shifterCylinder.set(DoubleSolenoid.Value.kForward);
		//arcadeDrive((-Controls.driverJoystick.getRawAxis(1)*reduction), (Controls.driverJoystick.getRawAxis(2)*reduction));
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
	
	/*if(driveState){
		dualStick();
	}
	else{
		slowMove(.5);
	}*/
	
	/*private void testForCorrectionMode() {
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
	}*/
	
}

