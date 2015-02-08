package org.usfirst.frc.team3167.control;

import org.usfirst.frc.team3167.drive.RobotKinematics;
import org.usfirst.frc.team3167.vision.VisionThread;

import edu.wpi.first.wpilibj.vision.AxisCamera;

public class RobotPositionEstimator
{
	
	private RobotKinematics position;
	private boolean seesTote = false;
	private VisionThread thread;
	
	public RobotPositionEstimator(AxisCamera camera)
	{
		position.x = 0;
		position.y = 0;
		position.theta = 0;
		thread = new VisionThread(VisionThread.LONG, camera, "\\home\\lvuser\\logo.jpg", this);
		thread.start();
	}
	
	public synchronized void setSeesTarget(boolean seesTote)
	{
		this.seesTote = seesTote;
	}
	
	public synchronized void setPosition(RobotKinematics position)
	{
		this.position = position;
	}
	
	public synchronized RobotKinematics GetCurrentPosition()
	{
		return position;
	}
	
	public synchronized boolean SeesTarget()
	{
		return seesTote;
	}
}
