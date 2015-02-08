package org.usfirst.frc.team3167.drive;

/**
 * Class for position control of an axis driven by a motor.
 * Includes bang-bang control with encoder feedback and homing functionality.
 * 
 * @author kloux
 *
 */

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Encoder;

import org.usfirst.frc.team3167.util.SecondOrderLimiter;
import org.usfirst.frc.team3167.util.BangBangController;
import org.usfirst.frc.team3167.util.DigitalSwitch;
import org.usfirst.frc.team3167.robot.RobotConfiguration;

public class BangBangAxis
{
	private enum HomeState
	{
		NotStarted,
		FastTowardSwitch,
		SlowAwayFromSwitch,
		SlowTowardSwitch,
		Homed
	}
	private HomeState homeState = HomeState.NotStarted;
	
	private final double homingSpeed;// [units/sec]
	private final double homingAccel;// [units/sec^2]
	private final DigitalSwitch homeSwitch;
	
	private final double normalSpeed;// [units/sec]
	private final double normalAccel;// [units/sec^2]
	
	private final double positionScale;// [units/encoder pulse]
	private final double homeSwitchPosition;// [units]
	private double position;
	private double cmdPosition;
	private double bufferedCmdPosition;
	
	private SecondOrderLimiter cmdGenerator;
	private BangBangController controller;
	
	private final Jaguar motor;
	private final Encoder encoder;
	private final CANJaguar canMotor;
	
	private final double normalStep;// [units]
	private final double homingStep;// [units]
	
	public BangBangAxis(Jaguar motor, double normalSpeed, double normalAccel,
			double homingSpeed, double homingAccel, DigitalSwitch homeSwitch,
			double positionScale, double homeSwitchPosition, double tolerance, boolean reverse)
	{
		this.motor = motor;
		this.normalSpeed = normalSpeed;
		this.normalAccel = normalAccel;
		this.homingSpeed = homingSpeed;
		this.homingAccel = homingAccel;
		this.homeSwitch = homeSwitch;
		this.positionScale = positionScale;
		this.homeSwitchPosition = homeSwitchPosition;
		bufferedCmdPosition = homeSwitchPosition;
		
		controller = new BangBangController(motor, tolerance, reverse);
		controller.SetSpeed(1.0);// Not actually speed, this just says we want to allow maximum command to jag if necessary
		
		encoder = null;// TODO:  Fix this!  If not using CAN Jag this won't work
		
		canMotor = null;
		
		normalStep = CalculateStep(normalSpeed);
		homingStep = CalculateStep(homingSpeed);
	}
	
	public BangBangAxis(CANJaguar motor, double normalSpeed, double normalAccel,
			double homingSpeed, double homingAccel, DigitalSwitch homeSwitch,
			double positionScale, double homeSwitchPosition, double tolerance, boolean reverse)
	{
		this.canMotor = motor;
		this.normalSpeed = normalSpeed;
		this.normalAccel = normalAccel;
		this.homingSpeed = homingSpeed;
		this.homingAccel = homingAccel;
		this.homeSwitch = homeSwitch;
		this.positionScale = positionScale;
		this.homeSwitchPosition = homeSwitchPosition;
		bufferedCmdPosition = homeSwitchPosition;
		
		controller = new BangBangController(motor, tolerance, reverse);
		controller.SetSpeed(1.0);// Not actually speed, this just says we want to allow maximum command to jag if necessary
		
		this.motor = null;
		encoder = null;
		
		normalStep = CalculateStep(normalSpeed);
		homingStep = CalculateStep(homingSpeed);
	}
	
	private double CalculateStep(double velocity)
	{
		return velocity / RobotConfiguration.frequency;
	}
	
	public void Update()
	{
		System.out.println("Digital Switch on Channel " + homeSwitch.getChannel() + ": " + homeSwitch.IsPressed());
		double cmdVel = (cmdPosition - GetPosition()) * RobotConfiguration.frequency;
		if (homeState != HomeState.Homed)
		{
			switch (homeState)
			{
			default:
			case NotStarted:
				cmdPosition = GetPosition();
				cmdGenerator = new SecondOrderLimiter(normalSpeed, normalAccel, RobotConfiguration.frequency);
				homeState = HomeState.FastTowardSwitch;
				
			case FastTowardSwitch:
				cmdVel = -normalStep;
				if (homeSwitch.IsPressed())
				{
					cmdGenerator = new SecondOrderLimiter(homingSpeed, homingAccel, RobotConfiguration.frequency);
					homeState = HomeState.SlowAwayFromSwitch;
				}
				break;
				
			case SlowAwayFromSwitch:
				cmdVel = homingStep;
				if (!homeSwitch.IsPressed())
					homeState = HomeState.SlowTowardSwitch;
				break;
				
			case SlowTowardSwitch:
				if (homeSwitch.HasJustBeenPressed())
				{
					homeState = HomeState.Homed;
					position = homeSwitchPosition;
					cmdPosition = bufferedCmdPosition;
					cmdVel = 0.0;
					cmdGenerator = new SecondOrderLimiter(normalSpeed, normalAccel, RobotConfiguration.frequency);
				}
				else
					cmdVel = -homingStep;
				break;
			}
		}
		
		cmdVel = cmdGenerator.Process(cmdVel);// TODO:  Fix this!
		cmdPosition += cmdVel;
		controller.DoControl(cmdPosition, GetPosition());
	}
	
	public double GetPosition()
	{
		if (motor != null)
		{
			return encoder.getRaw() * positionScale;// TODO:  Any scaling required here?
		}
		else if (canMotor != null)
		{
			return canMotor.getPosition() * positionScale;// TODO:  Scaling required here?
		}
		
		return 0.0;
	}
	
	public void SetCmdPosition(double cmdPosition)
	{
		if (homeState == HomeState.Homed)
			this.cmdPosition = cmdPosition;
		else
			bufferedCmdPosition = cmdPosition;
	}
}
