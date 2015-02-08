package org.usfirst.frc.team3167.networking;

/**
 * Class that will take the byte stream received by the thread class and interpret the 
 * position from it
 */
public class PiVision 
{
	private byte[] currentData;		// Our newest data from the pi
	
	/**
	 * Constructor
	 */
	public PiVision()
	{
		currentData = new byte[2048];
	}
	
	/**
	 * Method for the thread class to pass the byte stream just received from the pi to us
	 * @param data
	 */
	public void setData(byte[] data)
	{
		currentData = data;
	}
	
	/*public int getX()
	{
		// TODO: Interpret the data from the packet in order to extract the info it's sending
	}
	
	public int getY()
	{
		// TODO: Interpret the data from the packet in order to extract the info it's sending
	}
	
	public int getTheta()
	{
		// TODO: Interpret the data from the packet in order to extract the info it's sending
	}*/
}
