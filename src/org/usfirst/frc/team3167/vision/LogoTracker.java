package org.usfirst.frc.team3167.vision;

// OpenCV imports
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.KeyPoint;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.usfirst.frc.team3167.drive.RobotKinematics;
import org.opencv.video.Video;

import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.HSLImage;
import edu.wpi.first.wpilibj.image.MonoImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.RGBImage;
import edu.wpi.first.wpilibj.vision.AxisCamera;

/**
 * The main class for tracking a long tote, that is, one which we can expect
 * to have a FIRST logo within the robot's field of view while it is driving 
 * around.  The class must be passed a camera, and the file path of a template
 * logo in the constructor. Once an object is constructed, call the init() method
 * to do an analysis of the template image.  To use this object in the code,
 * call doAnalysis() to update the robot's current knowledge of its state (a new
 * image is analyzed).  Then, seesTote() can be called to determine if a tote is in
 * view or not, and getPosition will return the robot's position.
 * 
 */
public class LogoTracker extends Tracker
{
	static
	{
		//System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		System.load("/usr/local/lib/lib_OpenCV/java/lib" + Core.NATIVE_LIBRARY_NAME + ".so");
	}
	
	private AxisCamera camera; 				 	 // Camera that is on the forklift side of the robot
	private Mat template;	   				 	 	 // Mat holding the template image
	
	private boolean init;	   				 	 // Whether or not init() has been called yet

	private MatOfKeyPoint templateKeypoints; 	 // A mat of keypoints in the template image
	private FeatureDetector detector;		 	 // Feature detector object for SIFT
	private DescriptorMatcher matcher;       	 // SIFT descriptor matcher
	private DescriptorExtractor extractor;   	 // SIFT description extractor
	private Mat templateDescriptors;		 	 // Mat of descriptors extracted from the template image
	
	private MatOfKeyPoint imageKeypoints;		 	 // Mat of keypoints in the most recent image we viewed
	
	private ArrayList<DMatch> matches;		     // List of matches in most recent image
	private static final double LOGO_LENGTH = 3.0;  // [in]
	private static final double LOGO_HEIGHT = 3.0 + (5.0/8.0); // [in]
	
	private static Mat cameraMatrix;
	private static MatOfDouble dist;
	
	private VideoCapture vc;

	/**
	 * Create a LogoTracker
	 * 
	 * @param camera		The camera on the forklift side of the robot
	 * @param tempPath		The file path of the template image on the rRio
	 */
	public LogoTracker(AxisCamera camera, String tempPath)
	{
		this.camera = camera;													
		template = Highgui.imread(tempPath, 0);		// Use OpenCV to load the template image
		//System.out.println("Template dimensions: " + template.size());
		
		detector = FeatureDetector.create(FeatureDetector.SIFT);	// Create a SIFT detector
		matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_SL2);	// Match using a bruteforce method
		extractor = DescriptorExtractor.create(DescriptorExtractor.SIFT);	// Create a SIFT extractor
		templateDescriptors = new Mat();	
		templateKeypoints = new MatOfKeyPoint();
		
		imageKeypoints = null;				// No keypoints have been found yet
		matches = new ArrayList<DMatch>();	
		
		vc = new VideoCapture();
		vc.open("http://10.31.67.22/axis-media/media.amp");

		cameraMatrix = new MatOfDouble();
		cameraMatrix = MatOfDouble.zeros(3, 3, CvType.CV_64F);
		cameraMatrix.put(0, 0, 596.01281738);
		cameraMatrix.put(0, 2, 326.04810079);
		cameraMatrix.put(1, 1, 589.50994873);
		cameraMatrix.put(1, 2, 247.26001282);
		cameraMatrix.put(2, 2, 1.0);
		
		dist = new MatOfDouble(MatOfDouble.zeros(5, 1, CvType.CV_64F));
		dist.put(0, 0, -0.46737483);
		dist.put(0, 1,  0.26953156);
		dist.put(0, 2, -0.00093982);
		dist.put(0, 3, -0.00864156);
		dist.put(0, 4,  0.26339206);
		
		init = false;						// init() needs to be called before we can do anything
	}
	
	/**
	 * Function to initialize this object.  It will analyze the template image and store info to
	 * class variables.
	 */
	public void init()
	{
		// Check if this was called already
		if(init)
		{
			System.err.println("The tracker was already initialized");
			throw new IllegalStateException("The tracker was already initialized");
		}
		
		// Use the SIFT algorithm to find the keypoints and descriptors in the image
		detector.detect(template, templateKeypoints);
		extractor.compute(template, templateKeypoints, templateDescriptors);
		init = true;
		
	}

	public void doAnalysis()
	{
		// Check if init() was called already
		if(!init)
		{
			System.err.println("The tracker was not yet initialized");
			throw new IllegalStateException("The tracker was not yet initialized");
		}
		
		// Create an array to hold matches found in the new image
		DMatch[] matches = null;
		try 
		{
			// Apply SIFT to a new camera image with the template
			matches = doSIFT(getImage());
		} 
		catch (NIVisionException e) 
		{
			System.err.println(e.getMessage());
			e.printStackTrace();
		}
		
		// If no matches were found, or an exception was caught, make the matches array empty and stop
		if(matches == null)
		{
			this.matches = new ArrayList<DMatch>();
			return;
		}
		if(matches.length == 0)
		{
			this.matches = new ArrayList<DMatch>();
			return;
		}
		
		// Create an ArrayList to hold the matches that are clustered together
		ArrayList<DMatch> bestMatches = new ArrayList<DMatch>();
		
		/* For a point to be considered a "best" point, it must be in a cluster.  To determine if
		 * this is the case, check how many points are within a certain distance of it. */
		int minNeighborDistanceSquared = 500;
		int minNeighbors = 5;
		
		// Iterate over all matches
		for(DMatch match : matches)
		{
			int numNeighbors = 0;
			
			// Get the coordinates of the image keypoint for this match
			int x = (int)imageKeypoints.toArray()[match.trainIdx].pt.x;
			int y = (int)imageKeypoints.toArray()[match.trainIdx].pt.y;
			
			// Iterate over all matches
			for(DMatch match2 : matches)
			{
				// If match2 is match, they can't be neighbors since they're the same
				if(match != match2)
				{
					// Get the coordinates of the keypoint for match2
					int x2 = (int)imageKeypoints.toArray()[match2.trainIdx].pt.x;
					int y2 = (int)imageKeypoints.toArray()[match2.trainIdx].pt.y;
					
					// Calculuate the squared distance between the two keypoints
					double distanceSquared = (x2 - x)*(x2 - x) + (y2 - y)*(y2 - y);
					
					// If this exceeds a certain theshold, count match2 as a neighbore of match
					if(distanceSquared <= minNeighborDistanceSquared)
					{
						numNeighbors++;
					}
				}
				
				// If match has more than a certain number of neighbors, count it as a best match
				if(numNeighbors >= minNeighbors)
				{
					bestMatches.add(match);
				}
			}
		}
		
		// store best match to the class variable
		this.matches = bestMatches;
	}
	
	/**
	 * Determine if the tote is in the robot's view.
	 * 
	 * @return	Whether the logo is in the image
	 */
	public boolean seesTote()
	{
		// MinMatches in the minimum number of matching keypoints in the image to qualify as having a tote
		int minMatches = 10;
		return matches.size() >= minMatches;
	}
	 
	/**
	 * Return the position of the robot using solvePnP() from OpenCV
	 * @return
	 */
	public RobotKinematics getPosition()
	{
		
		Mat rvec = new Mat();
		Mat tvec = new Mat();
		
		MatOfPoint2f imagePoints = new MatOfPoint2f();
		List<Point> iPoints = new ArrayList<Point>();
		MatOfPoint3f objectPoints = findObjectPoints();
		
		for(int i = 0; i < matches.size(); i++)
		{
			DMatch match = matches.get(i);
			double [] point = new double[2];
			point[0] = imageKeypoints.toList().get(match.queryIdx).pt.x;
			point[1] = imageKeypoints.toList().get(match.queryIdx).pt.y;
			iPoints.add(new Point(point));
		}
		imagePoints.fromList(iPoints);
		
		/* Object points are the point on the logo in a 'logo coordinate system'.  It uses the same
		   units that we want to know the robot position in, and consist of an x,y coordinate 
		   system on the plane of the logo. */
		Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, dist, rvec, tvec);
		
		MatOfDouble R = new MatOfDouble();
		Calib3d.Rodrigues(rvec, R);
		
		MatOfDouble cameraRotationVector = new MatOfDouble();
		Calib3d.Rodrigues(R.t(), cameraRotationVector);
		
		MatOfDouble cameraTranslationVector = new MatOfDouble();
		Core.gemm(R, tvec, -1, tvec, 0, cameraTranslationVector);// Second tvec argument could be any matrix with the right size; the following argument must then be 0 to scale this dummy matrix away
		
		RobotKinematics pose = new RobotKinematics();
		pose.x = cameraTranslationVector.get(0, 0)[0];
		pose.y = cameraTranslationVector.get(1, 0)[0];
		pose.theta = cameraRotationVector.get(0,0)[0]*cameraRotationVector.get(0,0)[0]
				+ cameraRotationVector.get(1,0)[0]*cameraRotationVector.get(1,0)[0]
				+ cameraRotationVector.get(2,0)[0]*cameraRotationVector.get(2,0)[0];
		
		return pose;
	}
	
	/**
	 * Return the image currently seen by the camera as a Mat
	 * 
	 * @return The image seen by the camera as a Mat
	 * @throws NIVisionException
	 */
	private Mat getImage() throws NIVisionException
	{
		Mat image = new Mat();		
		vc.grab();
		vc.retrieve(image);
		// TODO: Convert the monoimage into an OpenCV mat
		// Read the image as a Mat using OpenCV
		//return Highgui.imread("\\home\\lvuser\\image.jpg", 0);
		return image;
	}
	
	/**
	 * Use SIFT to calculate the array of descriptor matches between a given image
	 * and the stored template
	 * 
	 * @param image	The image to analyze
	 * @return		The array of matches within the image			
	 */
	private DMatch[] doSIFT(Mat image)
	{
		// Detect image keypoints and store them
		MatOfKeyPoint imageKeypoints = new MatOfKeyPoint();
		detector.detect(image, imageKeypoints);
		
		// Compute the image descriptors for each keypoint and store them
		Mat imageDescriptors = new Mat();
		extractor.compute(image, imageKeypoints, imageDescriptors);
		
		// Using the two descriptor Mats, identify matches between the template and the image
		MatOfDMatch matches = new MatOfDMatch();
		matcher.match(templateDescriptors, imageDescriptors, matches);
				
		// Convert imageKeypoints and matchArray to arrays for the class
		this.imageKeypoints = imageKeypoints;
		DMatch[] matchArray = matches.toArray();
		
		// Sort the matches, so that the ones with the lowest distance are first
		Arrays.sort(matchArray, new Comparator<DMatch>()
			{
				public int compare(DMatch match1, DMatch match2)
				{
					if(match1.distance < match2.distance) return -1;
					else if(match1.distance > match2.distance) return 1;
					else return 0;
				}
			});
		
		// Take the top 30 matches, delete the rest
		DMatch[] tmp = matchArray;
		matchArray = new DMatch[30];
		System.arraycopy(tmp, 0, matchArray, 0, 30);
		
		return matchArray;
	}
	
	private MatOfPoint3f findObjectPoints()
	{
		MatOfPoint3f objectPoints = new MatOfPoint3f();
		List<Point3> oPoints = new ArrayList<Point3>();
		for(int i = 0; i < matches.size(); i++)
		{
			DMatch match = matches.get(i);
			
			double x = templateKeypoints.toArray()[match.queryIdx].pt.x;
			double y = templateKeypoints.toArray()[match.queryIdx].pt.y;
			
			double[] point = {(x/138.0)*LOGO_LENGTH, 1 - (y/124.0)*LOGO_HEIGHT, 0};
			//objectPoints.put(0, i, pointDouble);
			oPoints.add(new Point3(point));
		}
		
		objectPoints.fromList(oPoints);
		return objectPoints;
	} 
}
