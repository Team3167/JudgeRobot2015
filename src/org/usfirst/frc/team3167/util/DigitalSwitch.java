package org.usfirst.frc.team3167.util;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalSwitch extends Button
{
	private final DigitalInput di;
	
	public DigitalSwitch(int channel)
	{
		di = new DigitalInput(channel);
	}
	
	protected boolean GetState()
	{
		return di.get();
	}
	
	public int getChannel()
	{
		return di.getChannel();
	}
}
