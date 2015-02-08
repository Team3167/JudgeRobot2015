package org.usfirst.frc.team3167.autonomous;

import org.usfirst.frc.team3167.control.*;
import org.usfirst.frc.team3167.drive.HolonomicRobotDrive;
import org.usfirst.frc.team3167.drive.RobotKinematics;
import org.usfirst.frc.team3167.robot.RobotConfiguration;

/**
 * This class is intended to be a child class for a "PickUpTote" task.  Possibly needs to not itself be a Task?
 * Parent class will be responsible for communicating with Raspberry Pi and determining if it's OK for the
 * driver to be in control, or if we should continue to take control here.
 *
 * @author K. Loux
 */
public class MotionSubTask
{
	// Fields
	private RobotPositionEstimator posEstimator;
	private HolonomicRobotDrive drive;
	private MotionTaskController posCmdGenerator;
	private RobotKinematicsController posController;
	
	private static final double maxAccel = 50;// [in/sec^2]
	
	public MotionSubTask(HolonomicRobotDrive _drive, RobotPositionEstimator _posEstimator,
			RobotKinematics targetPosition, boolean smoothMotion)
	{
		drive = _drive;
		posEstimator = _posEstimator;
		
		RobotKinematics vel = new RobotKinematics();
		RobotKinematics acc = new RobotKinematics();
		vel.x = drive.GetMaxXVel();
		vel.y = drive.GetMaxYVel();
		vel.theta = drive.GetMaxOmega();
		acc.x = maxAccel;
		acc.y = maxAccel;
		acc.theta = maxAccel * drive.GetMaxOmega() * 2 / (drive.GetMaxXVel() + drive.GetMaxYVel());
		
		posCmdGenerator = new MotionTaskController(posEstimator.GetCurrentPosition(),
				targetPosition, vel, acc, RobotConfiguration.frequency, smoothMotion);
		posController = new RobotKinematicsController(1.0, 0.0, 200, RobotConfiguration.frequency);
	}
	
	protected void Update()
	{
		// position estimator should be updated in the class where it is owned (where it is created - here it is only
		// referenced - it's created in the part class)
		RobotKinematics velCmd = posController.DoControl(posCmdGenerator.Update(), posEstimator.GetCurrentPosition());
		
		// I think we can do without the velocity controller here - let's start out that way, anyway
		try
		{
			drive.Drive(velCmd.x, velCmd.y, velCmd.theta);
		}
		catch (Exception ex)
		{
			System.out.println("Failed to drive in MutionSubTask.Update:  " + ex.getMessage());
		}
	}
	
	protected boolean CloseToEndPosition(double errorLimit)
	{
		if (!posEstimator.SeesTarget())
			return false;
		return posCmdGenerator.ReachedTarget(posEstimator.GetCurrentPosition(), errorLimit);
	}
}
