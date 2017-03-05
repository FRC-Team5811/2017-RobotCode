package org.usfirst.frc.team5811.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

//some copypasta
// https://github.com/kauailabs/navxmxp/blob/master/roborio/java/navXMXP_Java_RotateToAngle/src/org/usfirst/frc/team2465/robot/Robot.java

public class NavXRotationalController extends PIDController {
	
	//PID Values
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;
	
	static final double kToleranceDegrees = 2.0f;

	public NavXRotationalController(PIDSource source, PIDOutput output) {
		super(kP, kI, kD, kF, source, output);
		setInputRange(-180.0f,  180.0f);
		setOutputRange(-1.0, 1.0);
		setContinuous(true);
		setAbsoluteTolerance(kToleranceDegrees);
	}

}
