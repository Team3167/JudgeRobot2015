package org.usfirst.frc.team3167.vision;

import java.awt.Dimension;
import java.util.ArrayList;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.File;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;

public class ImageDisplay extends JPanel
{

	private static final long serialVersionUID = 1L;
	
	private JFrame frame;
	private BufferedImage img1;
	private BufferedImage img2;
	
	private int x1;
	private int x2;
	private int y1;
	private int y2;
	
	private ArrayList<int[]> lines;
	
	public ImageDisplay(String img1Path, String img2Path)
	{
		loadImages(img1Path, img2Path);
		setPreferredSize(new Dimension(img1.getWidth() + img2.getWidth(), img2.getHeight()));
		lines = new ArrayList<int[]>();
		
		frame = new JFrame("images");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.add(this);
		frame.pack();
		frame.setResizable(false);
		frame.setLocationRelativeTo(null);
		frame.setVisible(true);
	}
	
	private void loadImages(String path1, String path2) 
	{
		try
		{
			img1 = ImageIO.read(new File(path1));
			img2 = ImageIO.read(new File(path2));
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
	}
	
	public synchronized void drawMatch(int x1, int y1, int x2, int y2)
	{		
		this.x1 = x1;
		this.y1 = y1;
		this.x2 = x2 + img1.getWidth();
		this.y2 = y2;
		
		int[] line = {this.x1, this.y1, this.x2, this.y2};
		lines.add(line);
		
		repaint();
	}
	
	@Override
	public synchronized void paintComponent(Graphics g)
	{
		super.paintComponent(g);
		g.drawImage(img1, 0, 0, null);
		g.drawImage(img2, img1.getWidth(null), 0, null);
		
		for(int[] line : lines)
		{
			g.setColor(java.awt.Color.BLUE);
			g.drawLine(line[0], line[1], line[2], line[3]);
			g.drawOval(line[0], line[1], 4, 4);
			g.drawOval(line[2], line[3], 4, 4);
		}
	}
}