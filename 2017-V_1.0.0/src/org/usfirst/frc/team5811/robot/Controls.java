package org.usfirst.frc.team5811.robot;

import org.usfirst.frc.team5811.robot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class Controls {
	//// CREATING BUTTONS

	// BUTTON MAPPING. REASON ITS HERE IS BECAUSE IT WAS WRONG AND THESE ARE
	// THE CORRECT VALUES
	// back button 9
	// left stick 11
	// right stick 12
	// start 10
	// lefttrigger 7
	// righttrigger 8
	// leftbumper 5
	// rightbumper 6
	// y 4
	// b 3
	// a 2
	// x 1
	
	// Controllers
	public static Joystick driverJoystick;
	public static Joystick manipulatorJoystick;

	// Buttons
	public static JoystickButton driverY;
	public static JoystickButton driverA;
	public static JoystickButton driverX;
	public static JoystickButton driverB;
	public static JoystickButton driverLeftBumper;
	public static JoystickButton driverRightBumper;
	public static JoystickButton driverLeftStick;
	public static JoystickButton driverRightStick;
	public static JoystickButton driverStart;
	public static JoystickButton driverBack;
	public static JoystickButton driverRightTrigger;
	public static JoystickButton driverLeftTrigger;
	
	public static JoystickButton manipulatorY;
	public static JoystickButton manipulatorA;
	public static JoystickButton manipulatorX;
	public static JoystickButton manipulatorB;
	public static JoystickButton manipulatorLeftBumper;
	public static JoystickButton manipulatorRightBumper;
	public static JoystickButton manipulatorLeftStick;
	public static JoystickButton manipulatorRightStick;
	public static JoystickButton manipulatorStart;
	public static JoystickButton manipulatorBack;
	public static JoystickButton manipulatorRightTrigger;
	public static JoystickButton manipulatorLeftTrigger;
	
	public Controls(){
		driverJoystick = new Joystick(RobotMap.joystickDriverSlot);
		manipulatorJoystick = new Joystick(RobotMap.joystickManipulatorSlot);

		// Buttons
		driverY = new JoystickButton(driverJoystick, 4);
		driverA = new JoystickButton(driverJoystick, 2);
		driverX = new JoystickButton(driverJoystick, 1);
		driverB = new JoystickButton(driverJoystick, 3);
		driverLeftBumper = new JoystickButton(driverJoystick, 5);
		driverRightBumper = new JoystickButton(driverJoystick, 6);
		driverLeftStick = new JoystickButton(driverJoystick, 11);
		driverRightStick = new JoystickButton(driverJoystick,12);
		driverStart = new JoystickButton(driverJoystick, 10);
		driverBack = new JoystickButton(driverJoystick, 9);
		driverRightTrigger = new JoystickButton(driverJoystick, 8);
		driverLeftTrigger = new JoystickButton(driverJoystick,7);
		
		manipulatorY = new JoystickButton(manipulatorJoystick, 4);
		manipulatorA = new JoystickButton(manipulatorJoystick, 2);
		manipulatorX = new JoystickButton(manipulatorJoystick, 1);
		manipulatorB = new JoystickButton(manipulatorJoystick, 3);
		manipulatorLeftBumper = new JoystickButton(manipulatorJoystick, 5);
		manipulatorRightBumper = new JoystickButton(manipulatorJoystick, 6);
		manipulatorLeftStick = new JoystickButton(manipulatorJoystick, 11);
		manipulatorRightStick = new JoystickButton(manipulatorJoystick,12);
		manipulatorStart = new JoystickButton(manipulatorJoystick, 10);
		manipulatorBack = new JoystickButton(manipulatorJoystick, 9);
		manipulatorRightTrigger = new JoystickButton(manipulatorJoystick, 8);
		manipulatorLeftTrigger = new JoystickButton(manipulatorJoystick,7);
		
		
		manipulatorX.toggleWhenPressed(new RunElevator());
		
	}

	
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
