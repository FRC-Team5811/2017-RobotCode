package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class ShifterPneumaticsHigh extends Subsystem{
	DoubleSolenoid shifterCylinder = RobotMap.shifterCylinder;
	
	public enum shiftState{
		
		highGear
	}
	public shiftState stateHigh = shiftState.highGear;
	
	public void initDefaultCommand() {}

	public void shiftHigh(){
		if(stateHigh != shiftState.highGear){
			shifterCylinder.set(DoubleSolenoid.Value.kForward);
			stateHigh = shiftState.highGear;
		}
	}
	public boolean isHigh(){
		return stateHigh == shiftState.highGear;
	}
	
	
	
	
}
