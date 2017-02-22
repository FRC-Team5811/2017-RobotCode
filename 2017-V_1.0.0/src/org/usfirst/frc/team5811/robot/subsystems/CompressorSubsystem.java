package org.usfirst.frc.team5811.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Compressor;

import org.usfirst.frc.team5811.robot.Map;

public class CompressorSubsystem extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	private Compressor compressor;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	
	public CompressorSubsystem() {
		// compressor port init
		compressor = new Compressor(Map.CompressorChannel);
		//this.setClosedLoopControlOff();
		this.setClosedLoopControlOn();
	}
	
	public void setClosedLoopControlOn(){
		compressor.setClosedLoopControl(true);
	}
	
	public void setClosedLoopControlOff(){
		compressor.setClosedLoopControl(false);
	}
}
