package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.Robot;
import org.usfirst.frc.team5811.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Elevator extends Subsystem {

    Victor elevatorMotor = RobotMap.elevatorMotor;
    
    public double speed = 0.3;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void run(){
    	double intakePower = Robot.power.elevator();
		if (intakePower >= 28) {
			elevatorMotor.set(0);
			Timer.delay(0.25);
		} else if (intakePower >= 24 && intakePower <= 28) {
				elevatorMotor.set(-0.15);
		} else if (intakePower < 24) {
				elevatorMotor.set(-0.3);
		}
    	
    	elevatorMotor.set(speed);
    }
    
    public void stop(){
    	elevatorMotor.set(0);
    }
}

