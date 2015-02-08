package org.usfirst.frc.team3167.vision;

import org.usfirst.frc.team3167.drive.RobotKinematics;

/**
 * Abstract base class for a vision tracking class.
 */
public abstract class Tracker 
{
	protected abstract void init();
	
	public abstract boolean seesTote();
	
	public abstract void doAnalysis();
	
	public abstract RobotKinematics getPosition();
}
