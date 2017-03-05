package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.Controls;
import org.usfirst.frc.team5811.robot.Robot;
import org.usfirst.frc.team5811.robot.RobotMap;
import org.usfirst.frc.team5811.robot.commands.RunDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem implements PIDOutput {
	
	public enum DriveMode {
		normal,
		holdStraight
	}
	
	private DriveMode mode = DriveMode.normal;
	
    //used in "correction" mode
	float rotationPos;
	double distance;
	
	//angle to set on next loop (set by pid stuff)
	double rotateToAngleRate = 0;
	double setAngle = 0;
	
	//Things the drivetrain cares about/controls
	AHRS navx = RobotMap.navx;
	RobotDrive drive = RobotMap.driveTrain;
	DoubleSolenoid shifter = RobotMap.shifterCylinder;
	Encoder encoder = RobotMap.driveEncoder;
	
	
	public DriveTrain() {
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		setDefaultCommand(new RunDrive());
	}
	
	public void runDrive(){
		
		double direction = -Controls.driverJoystick.getRawAxis(1);
		double rotation = -Controls.driverJoystick.getRawAxis(2);
		
		switch(mode){
		case normal:
			move(direction,rotation);
		case holdStraight:
			move(direction,rotateToAngleRate);
		}
		
		distance = encoder.getDistance();
	}
	
	//this is a method (instead of making 'mode' public) because we might need to do something
	//when switching drive modes - a class that just wants us to transition shouldn't know about
	//that stuff though
	public void transitionToMode(DriveMode toMode){
		switch(toMode){
		case normal:
			Robot.turnController.disable();
			break;
		case holdStraight:
			setAngle = navx.getAngle();
			Robot.turnController.enable();
			break;
		}
		mode = toMode;
	}
	
	public DriveMode getCurrentMode(){
		return mode;
	}
	
	private void shifterDelay(){
		int cycleCounterTele = 0;
		while(cycleCounterTele < 20){
			move(.3,0);
			cycleCounterTele++;
		}
	}
	
	private void move(double direction, double rotation){
		drive.arcadeDrive(direction, rotation);
	}
	
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

	//The NaxRotationController calls this most likely
	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
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

