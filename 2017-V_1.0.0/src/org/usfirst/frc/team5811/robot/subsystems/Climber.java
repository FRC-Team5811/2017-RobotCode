package org.usfirst.frc.team5811.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team5811.robot.RobotMap;

public class Climber extends Subsystem {
	
	private Victor leftMotor = RobotMap.leftClimberMotor;
	private Victor rightMotor = RobotMap.rightClimberMotor;
	private double speed;

	public Climber() {
		speed = 0.8;
	}

	@Override
	protected void initDefaultCommand() {}

	public void stop() {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

	public void set(double rate) {
		speed = rate;
	}

	public void inward() {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

	public void outward() {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

}
