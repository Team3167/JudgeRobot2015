package org.usfirst.frc.team3167.robot;

/**
 *  A class solely to hold global constants needed throughout the program, such as
 *  motor properties, pins that certain sensors are plugged into, or PID constants
 */
public class RobotConfiguration
{
	public static final double frequency = 60;// [Hz]
	
	// CAN Sync group for drive wheels
	public static final byte wheelCANSyncGroup = 1;
	
	// Wheel geometry
	public static final double wheelRadius = 2.0;// [in]
	public static final double wheelbase = 10.0;// [in]
	public static final double track = 20.0;// [in]
	public static final double rollerAngle = 45.0;// [deg]
	
	public static final double wheelGearboxRatio = 10.0;// [-]
	public static final double wheelMotorMaxSpeed = 3000.0;// [RPM]
	
	// Wheel control
	public static final double wheelKp = 1.0;
	public static final double wheelKi = 0.0;
	
	public static final int wheelEncoderPPR = 360;
	
	// CAN Node IDs
	public static final int leftFrontMotorID = 5;
	public static final int rightFrontMotorID = 6;
	public static final int leftRearMotorID = 2;
	public static final int rightRearMotorID = 1;
	public static final int narrowToteLiftMotorID = 3;
	public static final int wideToteLiftMotorID = 4;
	public static final int powerDistBoardID = 7;
	
	// Digital inputs
	public static final int narrowToteHomeChannel = 9;
	public static final int wideToteHomeChannel = 8;
	
	// Lift geometry
	public static final double narrowHomeSwitchHeight = 10;// [in]
	public static final double wideHomeSwitchHeight = 10;// [in]
} 
