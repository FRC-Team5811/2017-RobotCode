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
	public static Joystick joyStickLeft = new Joystick(Map.joystickDriverSlot);
	public static Joystick joyStickRight = new Joystick(Map.joystickManipulatorSlot);

	// Buttons
	public static JoystickButton logitechY = new JoystickButton(joyStickLeft, 4);
	public static JoystickButton logitechA = new JoystickButton(joyStickLeft, 2);
	public static JoystickButton logitechX = new JoystickButton(joyStickLeft, 1);
	public static JoystickButton logitechB = new JoystickButton(joyStickLeft, 3);
	public static JoystickButton logitechLeftBumper = new JoystickButton(joyStickLeft, 5);
	public static JoystickButton logitechRightBumper = new JoystickButton(joyStickLeft, 6);
	public static JoystickButton logitechLeftStickPress = new JoystickButton(joyStickLeft, 11);
	public static JoystickButton logitechRightStickPress = new JoystickButton(joyStickLeft,12);
	public static JoystickButton logitechStart = new JoystickButton(joyStickLeft, 10);
	public static JoystickButton logitechBack = new JoystickButton(joyStickLeft, 9);
	public static JoystickButton logitechRightTrigger = new JoystickButton(joyStickLeft, 8);
	public static JoystickButton logitechLeftTrigger = new JoystickButton(joyStickLeft,7);
	
	public static JoystickButton logitechY2 = new JoystickButton(joyStickRight, 4);
	public static JoystickButton logitechA2 = new JoystickButton(joyStickRight, 2);
	public static JoystickButton logitechX2 = new JoystickButton(joyStickRight, 1);
	public static JoystickButton logitechB2 = new JoystickButton(joyStickRight, 3);
	public static JoystickButton logitechLeftBumper2 = new JoystickButton(joyStickRight, 5);
	public static JoystickButton logitechRightBumper2 = new JoystickButton(joyStickRight, 6);
	public static JoystickButton logitechLeftStickPress2 = new JoystickButton(joyStickRight, 11);
	public static JoystickButton logitechRightStickPress2 = new JoystickButton(joyStickRight,12);
	public static JoystickButton logitechStart2 = new JoystickButton(joyStickRight, 10);
	public static JoystickButton logitechBack2 = new JoystickButton(joyStickRight, 9);
	public static JoystickButton logitechRightTrigger2 = new JoystickButton(joyStickRight, 8);
	public static JoystickButton logitechLeftTrigger2 = new JoystickButton(joyStickRight,7);
	

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
