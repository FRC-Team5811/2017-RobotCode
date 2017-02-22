package org.usfirst.frc.team5811.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team5811.interfaces.*;

public class Climber extends Subsystem implements BiDirectionalElement{
	
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

	@Override
	public void stop() {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

	@Override
	public void set(double rate) {
		speed = rate;
	}

	@Override
	public void inward() {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

	@Override
	public void outward() {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

}
