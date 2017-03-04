package org.usfirst.frc.team5811.robot.commands;

import org.usfirst.frc.team5811.robot.Robot;
import org.usfirst.frc.team5811.robot.subsystems.Pneumatics;

public class ToggleShifter {
	Pneumatics.WingsState oldState;

    public ToggleShifter() {
        // Use requires() here to declare subsystem dependencies
        //requires(Robot.shift);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	oldState = Robot.wings.state;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.wings.toggle();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return oldState != Robot.wings.state;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
