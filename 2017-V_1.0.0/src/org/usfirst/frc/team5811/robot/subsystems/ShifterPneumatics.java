package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShifterPneumatics {
	DoubleSolenoid shifterCylinder = RobotMap.shifterCylinder;
	
	public enum shiftState{
		lowGear,
		highGear
	}
	public shiftState state = shiftState.lowGear;
	
	public void initDefaultCommand() {}
	
	public void shiftLow(){
		if(state != shiftState.lowGear){
			shifterCylinder.set(DoubleSolenoid.Value.kReverse);
			state = shiftState.lowGear;
		}
	}
	public void shiftHigh(){
		if(state != shiftState.highGear){
			shifterCylinder.set(DoubleSolenoid.Value.kForward);
			state = shiftState.highGear;
		}
	}
	public boolean isLow(){
		return state == shiftState.lowGear;
	}
	public boolean isHigh(){
		return state == shiftState.highGear;
	}
	
	
	
}
