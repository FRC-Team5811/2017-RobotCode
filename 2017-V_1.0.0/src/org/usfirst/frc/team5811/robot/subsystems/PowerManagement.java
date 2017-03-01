
package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.RobotMap;

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
		return power.getCurrent(RobotMap.PDPElevatorChannel);
	}
	
	public double intake(){
		return power.getCurrent(RobotMap.PDPIntakeChannel);
	}
	
	public double frontRightDrive(){
		return power.getCurrent(RobotMap.PDPFrontRightMotorChannel);
	}
	
	public double backRightDrive(){
		return power.getCurrent(RobotMap.PDPBackRightMotorChannel);
	}
	
	public double frontLeftDrive(){
		return power.getCurrent(RobotMap.PDPFrontLeftMotorChannel);
	}
	
	public double backLeftDrive(){
		return power.getCurrent(RobotMap.PDPBackLeftMotorChannel);
	}
	
	public double climber1(){
		return power.getCurrent(RobotMap.PDPClimber1MotorChannel);
	}
	
	public double climber2(){
		return power.getCurrent(RobotMap.PDPClimber2MotorChannel);
	}

}
