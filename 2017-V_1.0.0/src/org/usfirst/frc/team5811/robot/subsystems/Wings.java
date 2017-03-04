package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Wings extends Subsystem {

	DoubleSolenoid cylinder = RobotMap.reservoirCylinder;
	
	public enum WingsState {
		expanded,
		contracted
	}
	
	public WingsState state = WingsState.contracted;

    public void initDefaultCommand() {
    }
    
    public void expand(){
    	if(state != WingsState.expanded){
    		cylinder.set(DoubleSolenoid.Value.kReverse);
    		state = WingsState.expanded;
    	}
    }
    
    public void contract(){
    	if(state != WingsState.contracted){
        	cylinder.set(DoubleSolenoid.Value.kForward);
        	state = WingsState.contracted;
    	}
    }
    
    public boolean isExpanded(){
    	return state == WingsState.expanded;
    }
    
    public boolean isContracted(){
    	return state == WingsState.contracted;
    }
    
    public WingsState toggle(){
    	switch(state){
    	case expanded:
    		contract();
    	case contracted:
    		expand();
    	}
    	return state;
    }
    
}

