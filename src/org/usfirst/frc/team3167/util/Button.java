package org.usfirst.frc.team3167.util;

public abstract class Button
{
	// Fields
	private boolean stateRequested = false;

	// Methods
    /**
	 * Returns true only once per click of the button.  Will not return true
	 * until the button (or buttons) has (have) been released and re-clicked.
	 *
	 * @return True if this is the first time we're requesting the state of the
	 * button and it is in fact pressed, false otherwise
	 */
	public boolean HasJustBeenPressed()
	{
		// If the button is pressed, and we haven't asked for the state yet,
		// return true and reset the stateRequested flag
		if (GetState() && !stateRequested)
		{
			stateRequested = true;
			return true;
		}
		else if (!GetState())
			stateRequested = false;

		return false;
	}

    // Method returns true as long as button is held down
    public boolean IsPressed()
    {
    	return GetState();
    }
    
    protected abstract boolean GetState();
}
