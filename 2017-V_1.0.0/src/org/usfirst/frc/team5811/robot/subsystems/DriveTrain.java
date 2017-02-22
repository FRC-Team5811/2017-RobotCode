package org.usfirst.frc.team5811.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {

	public DriveTrain() {
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}
	
	

}

/*

public void encoderMacro(float encValue){
rotationCount = 0;
if(rotationCount < encValue){
	driveMotors(1, 1);
}
}

public void encoderCreep(float encValue){
while(rotationCount >= 0.9 * encValue && rotationCount < encValue){
	driveMotors(0.3, 0.3);
}
driveMotors(.5,.5);
}
*/

/*

public void gotoBoilerLeftWhileMiddlePosition(){
	encoderMacro(000);
	turnMacro(-90);
	encoderMacro(000);
	turnMacro(-45);
	encoderCreep(000);
}

public void gotoBoilerRightWhileMiddlePosition(){
	encoderMacro(000);
	turnMacro(90);
	encoderMacro(000);
	turnMacro(45);
	encoderCreep(000);
}

public void gotoBoilerLeftWhileLeftPosition(){
	encoderMacro(000);
	turnMacro(-90);
	encoderMacro(000);
	turnMacro(-45);
	encoderCreep(000);
}

public void gotoBoilerRightWhileRightPosition(){
	encoderMacro(000);
	turnMacro(90);
	encoderMacro(000);
	turnMacro(45);
	encoderCreep(000);
}

public void gotoLoadingRightWhileMiddlePosition(){
	encoderMacro(000);
	turnMacro(90);
	encoderMacro(000);
	turnMacro(45);
	encoderCreep(000);
}

public void gotoLoadingLeftWhileMiddlePosition(){
	encoderMacro(000);
	turnMacro(-90);
	encoderMacro(000);
	turnMacro(-45);
	encoderCreep(000);
}

public void gotoLoadingRightWhileRightPosition(){
	encoderMacro(000);
	turnMacro(90);
	encoderMacro(000);
	turnMacro(45);
	encoderCreep(000);
}

public void gotoBoilerLeftFromMiddleGear(){  //Priority
	encoderMacro(-000);
	turnMacro(-120);
	encoderMacro(000);
}

public void gotoLoadingLeftWhileLeftPosition(){
	encoderMacro(000);
	turnMacro(-90);
	encoderMacro(000);
	turnMacro(-45);
	encoderCreep(000);
}
public void gotoBoilerRightFromMiddleGear(){  //Priority
	encoderMacro(-000);
	turnMacro(120);
	encoderMacro(000);
}

public void gotoLoadingRightFromMiddleGear(){
	encoderMacro(-000);
	turnMacro(120);
	encoderMacro(000);
}

public void gotoLoadingLeftFromMiddleGear(){
	encoderMacro(-000);
	turnMacro(-120);
	encoderMacro(000);
}

public void gearMiddle(){
	encoderMacro(000);
	encoderCreep(000);
}

public void gearLeftWhileLoading(){
	encoderMacro(-000);
	turnMacro(180);
	encoderMacro(000);
	encoderCreep(000);
}

public void returnGearLeftWhileLoading(){
	encoderMacro(-000);
	turnMacro(180);
	encoderMacro(000);
}

public void gearRightWhileLoading(){
	encoderMacro(-000);
	turnMacro(180);
	encoderMacro(000);
	encoderCreep(000);
}

public void returnGearRightWhileLoading(){
	encoderMacro(-000);
	turnMacro(180);
	encoderMacro(000);
}

public void gearLeftWhileBoiler(){
	encoderMacro(-000);
	turnMacro(180);
	encoderMacro(000);
	encoderCreep(000);
}

public void returnGearLeftWhileBoiler(){
	encoderMacro(-000);
	turnMacro(180);
	encoderMacro(000);
}

public void gearRightWhileBoiler(){
	encoderMacro(-000);
	turnMacro(180);
	encoderMacro(000);
	encoderCreep(000);
}

public void returnGearRightWhileBoiler(){
	encoderMacro(-000);
	turnMacro(180);
	encoderMacro(000);
}

public void hopperWhileBoilerLeft(){
	encoderMacro(-000);
	turnMacro(135);
	encoderMacro(000);
	turnMacro(-45);
	encoderMacro(000);
	turnMacro(45);
}

public void hopperWhileBoilerRight(){
	encoderMacro(-000);
	turnMacro(-135);
	encoderMacro(000);
	turnMacro(45);
	encoderMacro(000);
	turnMacro(-45);
}

public void hopperWhileLoadingLeft(){
	encoderMacro(-000);
	turnMacro(135);
	encoderMacro(000);
	turnMacro(-45);
	encoderMacro(000);
	turnMacro(45);
}

public void hopperWhileLoadingRight(){
	encoderMacro(-000);
	turnMacro(-135);
	encoderMacro(000);
	turnMacro(45);
	encoderMacro(000);
	turnMacro(-45);
}

public void returnHopperWhileBoilerLeft(){
	encoderMacro(-000);
	Timer.delay(1);
	turnMacro(180);
	encoderMacro(000);
	turnMacro(45);
	encoderCreep(000);
}

public void returnHopperWhileBoilerRight(){
	encoderMacro(-000);
	Timer.delay(1);
	turnMacro(180);
	encoderMacro(000);
	turnMacro(-45);
	encoderCreep(000);
}

public void returnHopperWhileLoadingLeft(){
	encoderMacro(-000);
	Timer.delay(1);
	turnMacro(180);
	encoderMacro(000);
	turnMacro(45);
	encoderCreep(000);
}

public void returnHopperWhileLoadingRight(){
	encoderMacro(-000);
	Timer.delay(1);
	turnMacro(180);
	encoderMacro(000);
	turnMacro(-45);
	encoderCreep(000);
}
*/