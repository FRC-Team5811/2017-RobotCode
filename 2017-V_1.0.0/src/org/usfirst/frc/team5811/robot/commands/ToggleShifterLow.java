package org.usfirst.frc.team5811.robot.commands;

import org.usfirst.frc.team5811.robot.Robot;
import org.usfirst.frc.team5811.robot.subsystems.ShifterPneumaticsLow;

import edu.wpi.first.wpilibj.command.Command;

public class ToggleShifterLow extends Command {
	
	ShifterPneumaticsLow.shiftState oldState;

    public ToggleShifterLow() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.shiftLow);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	oldState = Robot.shiftLow.state;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.shiftLow.shiftLow();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return oldState != Robot.shiftLow.state;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
