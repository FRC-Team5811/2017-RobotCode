package org.usfirst.frc.team5811.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team5811.interfaces.*;

public class Climber extends Subsystem {
	
	private Victor leftMotor;
	private Victor rightMotor;
	private double speed;

	public Climber(Victor left, Victor right) {
		leftMotor = left;
		rightMotor = right;
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
