package org.usfirst.frc.team5811.robot.subsystems;

import org.usfirst.frc.team5811.robot.RobotMap;
import org.usfirst.frc.team5811.robot.RobotStates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

//rotationCount = shooterRightEnc.get();
//rotationRate = shooterRightEnc.getRate();
//rotationPeriod = shooterRightEnc.getRaw();

public class Shooter extends Subsystem {
	
	Encoder shooterEncoder;
	Spark shooterRight;
	Spark shooterLeft;
	
	PIDController pid1;
	PIDController pid2;

	public Shooter() {
		// NAME, P, I, D
		//super("Shooter", 2.0, 0.0, 0.0);
		
		shooterRight = new Spark(RobotMap.rightShooterMotor);
		shooterLeft = new Spark(RobotMap.leftShooterMotor);
		
		//motor is inverted control
		shooterLeft.setInverted(true);
		
		// Encoder inits and instantiations
		shooterEncoder = new Encoder(
			RobotMap.shooterEncoderChannelA,
			RobotMap.shooterEncoderChannelB,
			true,
			Encoder.EncodingType.k4X
		);
		
		shooterEncoder.setMaxPeriod(1);
		shooterEncoder.setDistancePerPulse(36);
		shooterEncoder.setMinRate(10);
		shooterEncoder.setSamplesToAverage(32);
		
		//: The feedforward term is multiplied by the setpoint for
		//the PID controller so that it scales with the desired output speed.
		pid1 = new PIDController(0, 0, 0, 0, shooterEncoder, shooterLeft);
		pid2 = new PIDController(0, 0, 0, 0, shooterEncoder, shooterRight);
		
		pid1.enable();
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
	}
	
	public void enable(){
		pid1.enable();
		pid2.enable();
		
		pid1.setSetpoint(.62);
		pid2.setSetpoint(.62);
	}
	
	public void disable(){
		pid1.disable();
		pid2.disable();
	}
}

/*private void toggleShooterMotor() {
	
	// SHOOTER + BLOCK PNEUMATIC

	if (logitechRightBumper2.get()) {
		if (!wasPressedRightBumper) {
			shouldBeRunningShooter = !shouldBeRunningShooter;
		}
		wasPressedRightBumper = true;
	} else {
		wasPressedRightBumper = false;
	}

	if(rotationRate >= 19000){
		spinUpComplete = true;
	} else {
		spinUpComplete = false;
	}
	
	if(shouldBeRunningShooter && !spinUpComplete){
		shooterRight.set(.62);
		shooterLeft.set(-.62);
	
	} else if (shouldBeRunningShooter && spinUpComplete) {
		rotationRate = shooterRightEnc.getRate();
		shooterSpeedCorrection = (19200-rotationRate)/5000;   //.62 is 19200, .65 is 20000, .61 is 19000
		shooterRight.set(.65+shooterSpeedCorrection);
		shooterLeft.set(-.65+shooterSpeedCorrection);
		System.out.println("Rotation Rate: " + rotationRate);
		System.out.println("Power Correction: " + shooterSpeedCorrection);
		
	} else {
		shooterRight.set(0);
	    shooterLeft.set(0);
	}
	
}*/

/*public void shootAutonomous(double shootTime){
if(rotationRate >= 19000){
	spinUpComplete = true;
} else {
	spinUpComplete = false;
}

if(!spinUpComplete){
	shooterRight.set(.62);
	shooterLeft.set(-.62);
} else if (shouldBeRunningShooter && spinUpComplete) {
	rotationRate = shooterRightEnc.getRate();
	shooterSpeedCorrection = (19200-rotationRate)/5000;   //.62 is 19200, .65 is 20000, .61 is 19000
	shooterRight.set(.65+shooterSpeedCorrection);
	shooterLeft.set(-.65+shooterSpeedCorrection);
	System.out.println("Rotation Rate: " + rotationRate);
	System.out.println("Power Correction: " + shooterSpeedCorrection);
	
} else {
	shooterRight.set(0);
    shooterLeft.set(0);
}
}
cycleCounter++;
		 
		 
			distance = drive.getDistance();
		
			rotationCountForDrive = drive.get();
			rotationRateForDrive = drive.getRate();
		System.out.println(autoMode);
		switch(autoMode){
			case baseline: //Cross baseline
				encoderMacro(250);
			 
				if(cycleCounter < 250){
					driveMotors(1, 1);
				}
			
				break;
			case gearMiddle:
				gearMiddle();
				
				if(cycleCounter < 100){
					driveMotors(1, 1);
				}else if(cycleCounter < 350){
					driveMotors(0.3, 0.3);
				}else if(cycleCounter < 450){
					reservoirCylinder.set(DoubleSolenoid.Value.kForward);
					driveMotors(0, 0);
				}
	
				break;
			case gearMiddleBoilerLeft:
				gearMiddle();
				gotoBoilerLeftFromMiddleGear();
				break;
			case gearMiddleBoilerRight:
				gearMiddle();
				gotoBoilerRightFromMiddleGear();
				break;
			case gearBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				gearLeftWhileBoiler();
				break;
			case gearBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				gearLeftWhileBoiler();
				break;
			case gearBoilerRight:
				gotoBoilerRightWhileRightPosition();
				gearRightWhileBoiler();
				break;
			case gearBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				gearRightWhileBoiler();
				break;
			case shootOnlyBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				//shootAutonomous(0);
				break;
			case shootOnlyBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				//shootAutonomous(0);
				break;
			case shootOnlyBoilerRight:
				gotoBoilerRightWhileRightPosition();
				//shootAutonomous(0);
				break;
			case shootOnlyBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				//shootAutonomous(0);
				break;
			case hopperOnlyBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				break;
			case hopperOnlyBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				break;
			case hopperOnlyBoilerRight:
				gotoBoilerRightWhileRightPosition();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				break;
			case hopperOnlyBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				break;
			case gearMiddleShootBoilerLeft: //Priority
				gearMiddle();
				gotoBoilerLeftFromMiddleGear();
				//shootAutonomous(0);
				break;
			case gearMiddleShootBoilerRight: //Priority
				gearMiddle();
				gotoBoilerRightFromMiddleGear();
				//shootAutonomous(0);
				break;
			case gearShootBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				//shootAutonomous(0);
				break;
			case gearShootBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				//shootAutonomous(0);
				break;
			case gearShootBoilerRight:
				gotoBoilerRightWhileRightPosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				//shootAutonomous(0);
				break;
			case gearShootBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				//shootAutonomous(0);
				break;
			case hopperShootBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case hopperShootBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case hopperShootBoilerRight:
				gotoBoilerRightWhileRightPosition();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case hopperShootBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case gearMiddleHopperBoilerLeft:
				gearMiddle();
				gotoBoilerLeftFromMiddleGear();
				hopperWhileBoilerLeft();
				break;
			case gearMiddleHopperBoilerRight:
				gearMiddle();
				gotoBoilerRightFromMiddleGear();
				hopperWhileBoilerRight();
				break;
			case gearHopperBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				hopperWhileBoilerLeft();
				break;
			case gearHopperBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				hopperWhileBoilerLeft();
				break;
			case gearHopperBoilerRight:
				gotoBoilerRightWhileRightPosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				hopperWhileBoilerRight();
				break;
			case gearHopperBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				hopperWhileBoilerRight();
				break;
			case gearMiddleHopperShootBoilerLeft:
				gearMiddle();
				gotoBoilerLeftFromMiddleGear();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case gearMiddleHopperShootBoilerRight:
				gearMiddle();
				gotoBoilerRightFromMiddleGear();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case gearHopperShootBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case gearHopperShootBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case gearHopperShootBoilerRight:
				gotoBoilerRightWhileRightPosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case gearHopperShootBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case shootHopperShootBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				//shootAutonomous(0);
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case shootHopperShootBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				//shootAutonomous(0);
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case shootHopperShootBoilerRight:
				gotoBoilerRightWhileRightPosition();
				//shootAutonomous(0);
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case shootHopperShootBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				//shootAutonomous(0);
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case ultimateAutoGearMiddleBoilerLeft:
				gearMiddle();
				gotoBoilerLeftFromMiddleGear();
				//shootAutonomous(0);
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case ultimateAutoGearMiddleBoilerRight:
				gearMiddle();
				gotoBoilerRightFromMiddleGear();
				//shootAutonomous(0);
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case ultimateAutoBoilerLeft:
				gotoBoilerLeftWhileLeftPosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				//shootAutonomous(0);
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case ultimateAutoBoilerLeftWhileMiddle:
				gotoBoilerLeftWhileMiddlePosition();
				gearLeftWhileBoiler();
				returnGearLeftWhileBoiler();
				//shootAutonomous(0);
				hopperWhileBoilerLeft();
				returnHopperWhileBoilerLeft();
				//shootAutonomous(0);
				break;
			case ultimateAutoBoilerRight:
				gotoBoilerRightWhileRightPosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				//shootAutonomous(0);
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case ultimateAutoBoilerRightWhileMiddle:
				gotoBoilerRightWhileMiddlePosition();
				gearRightWhileBoiler();
				returnGearRightWhileBoiler();
				//shootAutonomous(0);
				hopperWhileBoilerRight();
				returnHopperWhileBoilerRight();
				//shootAutonomous(0);
				break;
			case loadingOnlyLeft:
				gotoLoadingLeftWhileLeftPosition();
				break;
			case loadingOnlyLeftWhileMiddle:
				gotoLoadingLeftWhileMiddlePosition();
				break;
			case loadingOnlyRight:
				gotoLoadingRightWhileRightPosition();
				break;
			case loadingOnlyRightWhileMiddle:
				gotoLoadingRightWhileMiddlePosition();
				break;
			case gearMiddleLoadingLeft:
				gearMiddle();
				gotoLoadingLeftFromMiddleGear();
				break;
			case gearMiddleLoadingRight:
				gearMiddle();
				gotoLoadingRightFromMiddleGear();
				break;
			case gearLoadingLeft:
				gotoLoadingLeftWhileLeftPosition();
				gearLeftWhileLoading();
				returnGearLeftWhileLoading();
				break;
			case gearLoadingLeftWhileMiddle:
				gotoLoadingLeftWhileMiddlePosition();
				gearLeftWhileLoading();
				returnGearLeftWhileLoading();
				break;
			case gearLoadingRight:
				gotoLoadingRightWhileRightPosition();
				gearRightWhileLoading();
				returnGearRightWhileLoading();
				break;
			case gearLoadingRightWhileMiddle:
				gotoLoadingRightWhileMiddlePosition();
				gearRightWhileLoading();
				returnGearRightWhileLoading();
				break;
			case hopperLoadingLeft:
				gotoLoadingLeftWhileLeftPosition();
				hopperWhileLoadingLeft();
				returnHopperWhileLoadingLeft();
				break;
			case hopperLoadingLeftWhileMiddle:
				gotoLoadingLeftWhileMiddlePosition();
				hopperWhileLoadingLeft();
				returnHopperWhileLoadingLeft();
				break;
			case hopperLoadingRight:
				gotoLoadingRightWhileRightPosition();
				hopperWhileLoadingRight();
				returnHopperWhileLoadingRight();
				break;
			case hopperLoadingRightWhileMiddle:
				gotoLoadingRightWhileMiddlePosition();
				hopperWhileLoadingRight();
				returnHopperWhileLoadingRight();
				break;
			case gearMiddleHopperLoadingLeft:
				gearMiddle();
				gotoLoadingLeftFromMiddleGear();
				hopperWhileLoadingLeft();
				returnHopperWhileLoadingLeft();
				break;
			case gearMiddleHopperLoadingRight:
				gearMiddle();
				gotoLoadingRightFromMiddleGear();
				hopperWhileLoadingRight();
				returnHopperWhileLoadingRight();
				break;
			case gearHopperLoadingLeft:
				gotoLoadingLeftWhileLeftPosition();
				gearLeftWhileLoading();
				returnGearLeftWhileLoading();
				hopperWhileLoadingLeft();
				returnHopperWhileLoadingLeft();
				break;
			case gearHopperLoadingLeftWhileMiddle:
				gotoLoadingLeftWhileMiddlePosition();
				gearLeftWhileLoading();
				returnGearLeftWhileLoading();
				hopperWhileLoadingLeft();
				returnHopperWhileLoadingLeft();
				break;
			case gearHopperLoadingRight:
				gotoLoadingRightWhileRightPosition();
				gearRightWhileLoading();
				returnGearRightWhileLoading();
				hopperWhileLoadingRight();
				returnHopperWhileLoadingRight();
				break;
			case gearHopperLoadingRightWhileMiddle:
				gotoLoadingRightWhileMiddlePosition();
				gearRightWhileLoading();
				returnGearRightWhileLoading();
				hopperWhileLoadingRight();
				returnHopperWhileLoadingRight();
				break;
			case turnMacroTest:
				turnMacro(30);
				break;
			case noStringNoMove:
				driveMotors(0,0);
				break;
			case test:
				encoderCreep(3735);
		}	



//**************DEFAULT CODE IN CASE OF AN L**************
		if((botPosition.equalsIgnoreCase("test") &&										
				(allianceColor.equalsIgnoreCase(null) || allianceColor.equalsIgnoreCase(null)) &&
				chooseBoilerOrLoading.equalsIgnoreCase(null) &&
				baselineCross.equalsIgnoreCase(null) &&
				gearPlacement.equalsIgnoreCase(null) &&
				shoot.equalsIgnoreCase(null) &&
				shootAfterHopper.equalsIgnoreCase(null) &&
				hopperPickup.equalsIgnoreCase(null))){	
			autoMode = RobotStates.test;
		}
		if((botPosition.equalsIgnoreCase(null) &&										
				(allianceColor.equalsIgnoreCase(null) || allianceColor.equalsIgnoreCase(null)) &&
				chooseBoilerOrLoading.equalsIgnoreCase(null) &&
				baselineCross.equalsIgnoreCase(null) &&
				gearPlacement.equalsIgnoreCase(null) &&
				shoot.equalsIgnoreCase(null) &&
				shootAfterHopper.equalsIgnoreCase(null) &&
				hopperPickup.equalsIgnoreCase(null))){	
			autoMode = RobotStates.noStringNoMove;
		}
		if((botPosition.equalsIgnoreCase("right") || botPosition.equalsIgnoreCase("left")) &&			
				(allianceColor.equalsIgnoreCase("blue") || allianceColor.equalsIgnoreCase("red")) &&
				(chooseBoilerOrLoading.equalsIgnoreCase("boiler") || chooseBoilerOrLoading.equalsIgnoreCase("loading")) &&
				baselineCross.equalsIgnoreCase("yes") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){	
			System.out.println("auto changed");
			autoMode = RobotStates.baseline;
		}
		if((botPosition.equalsIgnoreCase("middle") &&										
				(allianceColor.equalsIgnoreCase("blue") || allianceColor.equalsIgnoreCase("red")) &&
				chooseBoilerOrLoading.equalsIgnoreCase("nil") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no"))){	
			autoMode = RobotStates.gearMiddle;
		}
		
		//**************BOILER-BASED FUNCTIONS**************
		
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.shootOnlyBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.shootOnlyBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.shootOnlyBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.shootOnlyBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperOnlyBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperOnlyBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperOnlyBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperOnlyBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearShootBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearShootBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperShootBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperShootBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperShootBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperShootBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.shootHopperShootBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.shootHopperShootBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.shootHopperShootBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.shootHopperShootBoilerRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoGearMiddleBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoGearMiddleBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoBoilerLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoBoilerLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoBoilerRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("boiler") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("yes") &&
				shootAfterHopper.equalsIgnoreCase("yes") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.ultimateAutoBoilerRightWhileMiddle;
		}
		
		//**************LOADING-BASED FUNCTIONS**************
		
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.loadingOnlyLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.loadingOnlyLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.loadingOnlyRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.loadingOnlyRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleLoadingLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearMiddleLoadingRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearLoadingLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearLoadingLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearLoadingRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("no")){
			autoMode = RobotStates.gearLoadingRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperLoadingLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperLoadingLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperLoadingRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("nil") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.hopperLoadingRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperLoadingLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("middle") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearMiddleHopperLoadingRight;
		}
		if(botPosition.equalsIgnoreCase("left") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperLoadingLeft;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("red") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("left") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperLoadingLeftWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("right") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperLoadingRight;
		}
		if(botPosition.equalsIgnoreCase("middle") &&
				allianceColor.equalsIgnoreCase("blue") &&
				chooseBoilerOrLoading.equalsIgnoreCase("loading") &&
				baselineCross.equalsIgnoreCase("no") &&
				gearPlacement.equalsIgnoreCase("right") &&
				shoot.equalsIgnoreCase("no") &&
				shootAfterHopper.equalsIgnoreCase("no") &&
				hopperPickup.equalsIgnoreCase("yes")){
			autoMode = RobotStates.gearHopperLoadingRightWhileMiddle;
		}
		if(botPosition.equalsIgnoreCase("turn") &&
				allianceColor.equalsIgnoreCase("turn") &&
				chooseBoilerOrLoading.equalsIgnoreCase("turn") &&
				baselineCross.equalsIgnoreCase("turn") &&
				gearPlacement.equalsIgnoreCase("turn") &&
				shoot.equalsIgnoreCase("turn") &&
				shootAfterHopper.equalsIgnoreCase("turn") &&
				hopperPickup.equalsIgnoreCase("turn")){
			autoMode = RobotStates.turnMacroTest;
		}
*
*/
