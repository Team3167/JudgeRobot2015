package org.usfirst.frc.team3167.vision;

import org.usfirst.frc.team3167.control.RobotPositionEstimator;

import edu.wpi.first.wpilibj.vision.AxisCamera;

public class VisionThread extends Thread
{
	public static final byte WIDE = 0;
	public static final byte LONG = 1;
		
	private Tracker tracker;
	private RobotPositionEstimator manager;
		
	public VisionThread(byte mode, AxisCamera camera, String templatePath, RobotPositionEstimator manager)
	{
		super();
		this.manager = manager;
		if(mode == LONG)
		{
			tracker = new LogoTracker(camera, templatePath);
		}
		else if(mode == WIDE)
		{
			tracker = new WideTracker(camera, templatePath);
		}
	}
	
	public void run()
	{
		if(tracker.seesTote())
		{
			manager.setSeesTarget(true);
			tracker.doAnalysis();
			manager.setPosition(tracker.getPosition());
		}
		else
		{
			manager.setSeesTarget(false);
		}
	}
	
}
