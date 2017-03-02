package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.RobotMap;
import org.usfirst.frc.team5811.robot.RobotStates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

//rotationCount = shooterRightEnc.get();
//rotationRate = shooterRightEnc.getRate();
//rotationPeriod = shooterRightEnc.getRaw();

public class Shooter extends Subsystem {
	
	Encoder shooterEncoder = RobotMap.shooterEncoder;
	Spark shooterRight = RobotMap.rightShooterMotor;
	Spark shooterLeft = RobotMap.leftShooterMotor;
	
	PIDController pid1;
	PIDController pid2;

	public Shooter() {
		// NAME, P, I, D
		//super("Shooter", 2.0, 0.0, 0.0);
		
		//: The feedforward term is multiplied by the setpoint for
		//the PID controller so that it scales with the desired output speed.
		//pid1 = new PIDController(0, 0, 0, 0, shooterEncoder, shooterLeft);
		//pid2 = new PIDController(0, 0, 0, 0, shooterEncoder, shooterRight);
		
		//pid1.enable();
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
	}
	
	public void enable(){
		//pid1.enable();
		//pid2.enable();
		
		//pid1.setSetpoint(.62);
		//pid2.setSetpoint(.62);
	}
	
	public void disable(){	
		//pid1.disable();
		//pid2.disable();
	}
	
	public void run(){
		
		double rotationRate = shooterEncoder.getRate();
		if(rotationRate >= 19000){
			shooterRight.set(.62);
			shooterLeft.set(-.62);
		}else{
			double shooterSpeedCorrection = (19200-rotationRate)/5000;   //.62 is 19200, .65 is 20000, .61 is 19000
			shooterRight.set(.65+shooterSpeedCorrection);
			shooterLeft.set(-.65+shooterSpeedCorrection);
			System.out.println("Rotation Rate: " + rotationRate);
			System.out.println("Power Correction: " + shooterSpeedCorrection);
		}
	}
	
	public void stop(){
		shooterRight.set(0);
		shooterLeft.set(0);
	}
}

/*public void shootAutonomous(double shootTime){
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
*/
