package org.usfirst.frc.team5811.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team5811.robot.RobotMap;
import org.usfirst.frc.team5811.robot.commands.RunClimber;

public class Climber extends Subsystem {
	
	private Victor leftMotor = RobotMap.leftClimberMotor;
	private Victor rightMotor = RobotMap.rightClimberMotor;

	public Climber() {
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new RunClimber());
	}

	public void stop() {
		leftMotor.set(0);
		rightMotor.set(0);
	}

	public void run(double rate) {
		leftMotor.set(rate);
		rightMotor.set(rate);
	}

}
