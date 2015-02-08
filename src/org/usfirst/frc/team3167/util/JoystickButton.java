package org.usfirst.frc.team3167.util;

// WPI imports
import edu.wpi.first.wpilibj.Joystick;

/**
 * Class for handling joystick button presses.  Without special handling, button
 * presses may result in unexpected behavior due to repeated processing (cRIO
 * runs at fast rate - calling methods once per click requires this object or
 * similar code).
 *
 * @author K. Loux
 */
public class JoystickButton extends Button
{
    // Fields
	private final Joystick stick;
	private final int button;

	// Methods
	/**
	 * Constructor for buttons located on only one joystick.
	 *
	 * @param _stick	Joystick object from which button is read
	 * @param _button	Button number (which button on the joystick)
	 */
	public JoystickButton(Joystick _stick, int _button)
	{
		// Assign the joystick button we are checking to the local fields
		stick = _stick;
		button = _button;
	}

    // Method returns true as long as button is held down
	/**
	 * Returns true as long as the button is being pressed.  Equivalent to
	 * Joystick.getRawButton() method in libWPI.
	 *
	 * @return True if the button is pressed, false otherwise
	 */
    protected boolean GetState()
    {
		return stick.getRawButton(button);
    }
}