package org.usfirst.frc.team5811.robot;

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
	public static Joystick joyStickLeft;
	public static Joystick joyStickRight;

	// Buttons
	public static JoystickButton logitechY;
	public static JoystickButton logitechA;
	public static JoystickButton logitechX;
	public static JoystickButton logitechB;
	public static JoystickButton logitechLeftBumper;
	public static JoystickButton logitechRightBumper;
	public static JoystickButton logitechLeftStickPress;
	public static JoystickButton logitechRightStickPress;
	public static JoystickButton logitechStart;
	public static JoystickButton logitechBack;
	public static JoystickButton logitechRightTrigger;
	public static JoystickButton logitechLeftTrigger;
	
	public static JoystickButton logitechY2;
	public static JoystickButton logitechA2;
	public static JoystickButton logitechX2;
	public static JoystickButton logitechB2;
	public static JoystickButton logitechLeftBumper2;
	public static JoystickButton logitechRightBumper2;
	public static JoystickButton logitechLeftStickPress2;
	public static JoystickButton logitechRightStickPress2;
	public static JoystickButton logitechStart2;
	public static JoystickButton logitechBack2;
	public static JoystickButton logitechRightTrigger2;
	public static JoystickButton logitechLeftTrigger2;
	
	public Controls(){
		joyStickLeft = new Joystick(RobotMap.joystickDriverSlot);
		joyStickRight = new Joystick(RobotMap.joystickManipulatorSlot);

		// Buttons
		logitechY = new JoystickButton(joyStickLeft, 4);
		logitechA = new JoystickButton(joyStickLeft, 2);
		logitechX = new JoystickButton(joyStickLeft, 1);
		logitechB = new JoystickButton(joyStickLeft, 3);
		logitechLeftBumper = new JoystickButton(joyStickLeft, 5);
		logitechRightBumper = new JoystickButton(joyStickLeft, 6);
		logitechLeftStickPress = new JoystickButton(joyStickLeft, 11);
		logitechRightStickPress = new JoystickButton(joyStickLeft,12);
		logitechStart = new JoystickButton(joyStickLeft, 10);
		logitechBack = new JoystickButton(joyStickLeft, 9);
		logitechRightTrigger = new JoystickButton(joyStickLeft, 8);
		logitechLeftTrigger = new JoystickButton(joyStickLeft,7);
		
		logitechY2 = new JoystickButton(joyStickRight, 4);
		logitechA2 = new JoystickButton(joyStickRight, 2);
		logitechX2 = new JoystickButton(joyStickRight, 1);
		logitechB2 = new JoystickButton(joyStickRight, 3);
		logitechLeftBumper2 = new JoystickButton(joyStickRight, 5);
		logitechRightBumper2 = new JoystickButton(joyStickRight, 6);
		logitechLeftStickPress2 = new JoystickButton(joyStickRight, 11);
		logitechRightStickPress2 = new JoystickButton(joyStickRight,12);
		logitechStart2 = new JoystickButton(joyStickRight, 10);
		logitechBack2 = new JoystickButton(joyStickRight, 9);
		logitechRightTrigger2 = new JoystickButton(joyStickRight, 8);
		logitechLeftTrigger2 = new JoystickButton(joyStickRight,7);
		
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
