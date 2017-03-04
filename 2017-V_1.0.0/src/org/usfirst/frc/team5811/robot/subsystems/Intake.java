package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.Robot;
import org.usfirst.frc.team5811.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	Victor intake = RobotMap.intakeMotor;
	public double speed = -.85;

    public void initDefaultCommand() {}
    
    public void run(){
    	double intakeCurrent = Robot.power.intake();
		if (intakeCurrent >= 28) {
			intake.set(0);
			Timer.delay(0.25);
			if (intakeCurrent >= 24 && intakeCurrent <= 28) {
				intake.set(-0.6);
			} else if (intakeCurrent < 24) {
				intake.set(-0.85);
			}
		}
		intake.set(speed);
    }
    
    public void stop(){
    	intake.set(0);
    }
}

