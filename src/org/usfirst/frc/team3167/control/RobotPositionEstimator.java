package org.usfirst.frc.team3167.control;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

import org.usfirst.frc.team3167.drive.RobotKinematics;
import org.usfirst.frc.team3167.robot.RobotConfiguration;

import edu.wpi.first.wpilibj.vision.AxisCamera;

public class RobotPositionEstimator implements Runnable
{
	private ProcessBuilder builder;
	private RobotKinematics position;
	private boolean seesTote;
	private Thread thread;
	private DatagramSocket sender;
	private DatagramSocket receiver;
	private InetAddress localhost;
	private byte[] buffer;
	
	public RobotPositionEstimator(AxisCamera camera)
	{
		position.x = 0;
		position.y = 0;
		position.theta = 0;
		seesTote = false;
		
		try 
		{
			sender = new DatagramSocket();
			receiver = new DatagramSocket(RobotConfiguration.javaPort);
		}
		catch (SocketException e)
		{
			e.printStackTrace();
		}
		
		try 
		{
			localhost = InetAddress.getByName(null);
		} 
		catch (UnknownHostException e)
		{
			e.printStackTrace();
		}
		buffer = new byte[512];
		
		builder = new ProcessBuilder(RobotConfiguration.pythonCommandLinux, RobotConfiguration.pythonFilePath);
		try 
		{
			builder.start();
		} 
		catch (IOException e)
		{
			e.printStackTrace();
		}
		
		thread = new Thread(this);
		thread.start();
	}
	
	public void run()
	{
		boolean running = true;
		while(running)
		{
			sendToPython("GO");
			String message = listenToPython();
			if(message.trim().startsWith("YES"))
			{
				sawTarget(true);
			}
			else if(message.startsWith("NO"))
			{
				sawTarget(false);
			}
			else if(message.startsWith("POS"))
			{
				double x = Double.parseDouble(listenToPython());
				double y = Double.parseDouble(listenToPython());
				double theta = Double.parseDouble(listenToPython());
				
				RobotKinematics position = new RobotKinematics();
				position.x = x;
				position.y = y;
				position.theta = theta;
				setPosition(position);
			}
			else if(message.startsWith("EXIT"))
			{
				running = false;
			}
		}
	}
	
	public String listenToPython()
	{
		DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
		try 
		{
			receiver.receive(packet);
		} 
		catch (IOException e)
		{
			e.printStackTrace();
		}
		return new String(packet.getData());
	}
	
	public void sendToPython(String message)
	{
		byte[] data = message.getBytes();
		DatagramPacket packet = new DatagramPacket(data, data.length, localhost, RobotConfiguration.pythonPort);
		try 
		{
			sender.send(packet);
		} 
		catch (IOException e) 
		{
			e.printStackTrace();
		}
	}
	
	public synchronized void sawTarget(boolean seesTote)
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
