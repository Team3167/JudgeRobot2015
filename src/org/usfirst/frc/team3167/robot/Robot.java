
package org.usfirst.frc.team3167.robot;

import org.usfirst.frc.team3167.autonomous.TaskManager;
import org.usfirst.frc.team3167.drive.HolonomicRobotDrive;
import org.usfirst.frc.team3167.drive.Lift;
import org.usfirst.frc.team3167.drive.Wheel;
import org.usfirst.frc.team3167.util.Conversions;
import org.usfirst.frc.team3167.util.DigitalSwitch;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{
	private HolonomicRobotDrive drive;
	private Lift narrowLift = new Lift(RobotConfiguration.narrowToteLiftMotorID,
			new DigitalSwitch(RobotConfiguration.narrowToteHomeChannel),
			RobotConfiguration.narrowHomeSwitchHeight);
	private Lift wideLift = new Lift(RobotConfiguration.wideToteLiftMotorID,
			new DigitalSwitch(RobotConfiguration.wideToteHomeChannel),
			RobotConfiguration.wideHomeSwitchHeight);
	
	// TODO:  Add cameras and trackers
	
	private TaskManager taskManager = new TaskManager();
	private Joystick driveJoystick = new Joystick(0);

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
    {
    	BuildRobotDrive();
    	// TODO:  Create all the stuff here
    }
    
    private void DoCommonUpdates()
    {
    	taskManager.DoCurrentTask();
    	if (taskManager.OkToDrive())
    	{
    		try
    		{
    			drive.Drive(driveJoystick);
    		}
    		catch (Exception ex)
    		{
    			System.out.println("Failed to drive: " + ex.getMessage());
    		}
    	}
    	
    	narrowLift.Update();
    	wideLift.Update();
    	
    	// TODO:  Update all of our common objects
    	// Both target trackers
    	// Anything else?
    	
    	// CAN objects - this line sends the commands
    	CANJaguar.updateSyncGroup(RobotConfiguration.wheelCANSyncGroup);
    }
    
    public void autonomousInit()
    {
    	System.out.println("autonomousInit");
    	
    	taskManager.ClearAllTasks();
    	
    	if (true)
    	{
    		//taskManager.AddTask(new DriveForwardTask());// TODO:  What do we want to do for autonomous?
    	}
    	// TODO Add more options + ways to select
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic()
    {
    	DoCommonUpdates();
    	// TODO:  Do stuff here
    }
    
    public void teleopInit()
    {
    	System.out.println("teleopInit");
    	
    	taskManager.ClearAllTasks();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() 
    {
    	DoCommonUpdates();
    	// TODO:  Respond to button presses, etc.
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic()
    {
    }
    
    private void BuildRobotDrive()
    {
    	drive = new HolonomicRobotDrive(RobotConfiguration.frequency);
    	
    	double maxWheelSpeed = RobotConfiguration.wheelMotorMaxSpeed
    			/ RobotConfiguration.wheelGearboxRatio
    			* Conversions.RPMToRadPerSec;// [rad/sec]
    	double halfTrack = RobotConfiguration.track;// [in]
    	double halfWheelbase = RobotConfiguration.wheelbase;// [in]
    	double leftWheelAxisX = 1;
    	double rightWheelAxisX = -leftWheelAxisX;
    	double rollerAngle1 = -RobotConfiguration.rollerAngle;
    	double rollerAngle2 = -rollerAngle1;
    	
    	// Left front
    	drive.AddWheel(new Wheel(-halfTrack, halfWheelbase, leftWheelAxisX, 0, rollerAngle1,
    			RobotConfiguration.wheelRadius, /*RobotConfiguration.wheelGearboxRatio*/1,
    			maxWheelSpeed, RobotConfiguration.leftFrontMotorID,
    			RobotConfiguration.wheelKp, RobotConfiguration.wheelKi, RobotConfiguration.wheelEncoderPPR));
    	
    	// Right front
    	drive.AddWheel(new Wheel(halfTrack, halfWheelbase, rightWheelAxisX, 0, rollerAngle2,
    			RobotConfiguration.wheelRadius, /*RobotConfiguration.wheelGearboxRatio*/1,
    			maxWheelSpeed, RobotConfiguration.rightFrontMotorID, 
    			RobotConfiguration.wheelKp, RobotConfiguration.wheelKi, RobotConfiguration.wheelEncoderPPR));
    	
    	// Left rear
    	drive.AddWheel(new Wheel(-halfTrack, -halfWheelbase, leftWheelAxisX, 0, rollerAngle2,
    			RobotConfiguration.wheelRadius, /*RobotConfiguration.wheelGearboxRatio*/1,
    			maxWheelSpeed, RobotConfiguration.leftRearMotorID,
    			RobotConfiguration.wheelKp, RobotConfiguration.wheelKi, RobotConfiguration.wheelEncoderPPR));
    	
    	// Right rear
    	drive.AddWheel(new Wheel(halfTrack*2, -halfWheelbase, rightWheelAxisX, 0, rollerAngle1,// TODO:  Why do I need the *2?
    			RobotConfiguration.wheelRadius, /*RobotConfiguration.wheelGearboxRatio*/1,
    			maxWheelSpeed, RobotConfiguration.rightRearMotorID,
    			RobotConfiguration.wheelKp, RobotConfiguration.wheelKi, RobotConfiguration.wheelEncoderPPR));
    	
    	// Add acceleration limits
    	drive.SetFrictionCoefficient(0.7);// Creates acceleration limit

    	// Set the deadband for the joysticks
    	drive.SetDeadband(0.1);// horizontal deadband
    	drive.SetMinimumOutput(0.1);// vertical deadband
    	
    	drive.Initialize();
    }
    
}
