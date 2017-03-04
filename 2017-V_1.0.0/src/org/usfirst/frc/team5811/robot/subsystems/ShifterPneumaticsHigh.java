package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShifterPneumaticsHigh extends Subsystem{
	DoubleSolenoid shifterCylinder = RobotMap.shifterCylinder;
	
	public enum shiftState{
		
		highGear
	}
	public shiftState state = shiftState.highGear;
	
	public void initDefaultCommand() {}

	public void shiftHigh(){
		if(state != shiftState.highGear){
			shifterCylinder.set(DoubleSolenoid.Value.kForward);
			state = shiftState.highGear;
		}
	}
	public boolean isHigh(){
		return state == shiftState.highGear;
	}
	
	
	
	
}
