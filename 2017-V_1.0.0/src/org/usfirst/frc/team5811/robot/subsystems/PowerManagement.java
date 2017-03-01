
package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.Map;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;


public class PowerManagement extends Subsystem {
	
	// power distribution panel
	PowerDistributionPanel power;

	public PowerManagement(PowerDistributionPanel panel) {
		power = panel;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
	}
	
	public double elevator(){
		return power.getCurrent(Map.PDPElevatorChannel);
	}
	
	public double intake(){
		return power.getCurrent(Map.PDPIntakeChannel);
	}
	
	public double frontRightDrive(){
		return power.getCurrent(Map.PDPFrontRightMotorChannel);
	}
	
	public double backRightDrive(){
		return power.getCurrent(Map.PDPBackRightMotorChannel);
	}
	
	public double frontLeftDrive(){
		return power.getCurrent(Map.PDPFrontLeftMotorChannel);
	}
	
	public double backLeftDrive(){
		return power.getCurrent(Map.PDPBackLeftMotorChannel);
	}
	
	public double climber1(){
		return power.getCurrent(Map.PDPClimber1MotorChannel);
	}
	
	public double climber2(){
		return power.getCurrent(Map.PDPClimber2MotorChannel);
	}

}
