package org.usfirst.frc.team3167.vision;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import javax.imageio.ImageIO;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.KeyPoint;
import org.opencv.highgui.Highgui;

public class LogoTracker 
{
	private FeatureDetector detector;
	private MatOfKeyPoint templateKeypoints;
	private DescriptorMatcher matcher;
	private DescriptorExtractor extractor;
	private Mat templateDescriptors;
	
	private ImageDisplay display;
		
	private boolean init = false;
	
	public LogoTracker()
	{
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		detector = FeatureDetector.create(FeatureDetector.SIFT);
		matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_SL2);
		templateDescriptors = new Mat();
		extractor = DescriptorExtractor.create(DescriptorExtractor.SIFT);
		templateKeypoints = new MatOfKeyPoint();
		
		display = new ImageDisplay("C:\\Users\\Owner\\Downloads\\FIRSTLogo.jpg",
				"C:\\Users\\Owner\\Downloads\\sample3.jpg");
	}
	
	public static void main(String args[])
	{
		LogoTracker tracker = new LogoTracker();
		tracker.train("C:\\Users\\Owner\\Downloads\\FIRSTLogo.jpg");
		tracker.analyze("C:\\Users\\Owner\\Downloads\\sample3.jpg");
	}
	
	public void train(String templatePath)
	{
		if(init)
		{
			System.err.println("You already trained the tracker for a target.");
			return;
		}
		Mat template = Highgui.imread(templatePath, Highgui.CV_LOAD_IMAGE_GRAYSCALE);
		detector.detect(template, templateKeypoints);
		extractor.compute(template, templateKeypoints, templateDescriptors);
		init = true;
	}
	
	public void analyze(String filename)
	{
		if(!init)
		{
			System.err.println("You must train the tracker first by calling train()");
			return;
		}
		Mat image = Highgui.imread(filename, Highgui.CV_LOAD_IMAGE_GRAYSCALE);
		MatOfKeyPoint imageKeypoints = new MatOfKeyPoint();
		detector.detect(image, imageKeypoints);
		
		Mat imageDescriptors = new Mat();
		extractor.compute(image, imageKeypoints, imageDescriptors);
		
		MatOfDMatch matches = new MatOfDMatch();
		matcher.match(templateDescriptors, imageDescriptors, matches);
				
		KeyPoint[] keypointArray = imageKeypoints.toArray();
		DMatch[] matchArray = matches.toArray();
		
		Arrays.sort(matchArray, new Comparator<DMatch>()
			{
				public int compare(DMatch match1, DMatch match2)
				{
					if(match1.distance < match2.distance) return -1;
					else if(match1.distance > match2.distance) return 1;
					else return 0;
				}
			});
		
		/*for(int i = 0; i < matchArray.length; i++)
		{
			System.out.println(matchArray[i]);
		}*/
		
		DMatch[] tmp = matchArray;
		matchArray = new DMatch[30];
		System.arraycopy(tmp, 0, matchArray, 0, 30);
		
		/*for(int i = 0; i < matchArray.length; i++)
		{
			System.out.println(keypointArray[matchArray[i].trainIdx]);
		}*/
		
		for(int i = 0; i < matchArray.length; i++)
		{
			int x2 = (int)keypointArray[matchArray[i].trainIdx].pt.x;
			int y2 = (int)keypointArray[matchArray[i].trainIdx].pt.y;
			
			int x1 = (int)templateKeypoints.toArray()[matchArray[i].queryIdx].pt.x;
			int y1 = (int)templateKeypoints.toArray()[matchArray[i].queryIdx].pt.y;
			
			
			display.drawMatch(x1, y1, x2, y2);
		}
		
		ArrayList<DMatch> bestMatches = new ArrayList<DMatch>();
		
	}
	
}