import numpy as np
import cv2
from matplotlib import pyplot as plt
import time

def drawMatches(img1, kp1, img2, kp2, matches, avg_x, avg_y, length):
    """
    My own implementation of cv2.drawMatches as OpenCV 2.4.9
    does not have this function available but it's supported in
    OpenCV 3.0.0

    This function takes in two images with their associated 
    keypoints, as well as a list of DMatch data structure (matches) 
    that contains which keypoints matched in which images.

    An image will be produced where a montage is shown with
    the first image followed by the second image beside it.

    Keypoints are delineated with circles, while lines are connected
    between matching keypoints.

    img1,img2 - Grayscale images
    kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
              detection algorithms
    matches - A list of matches of corresponding keypoints through any
              OpenCV keypoint matching algorithm
    """

    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1,:] = np.dstack([img1, img1, img1])

    # Place the next image to the right of it
    out[:rows2,cols1:cols1+cols2,:] = np.dstack([img2, img2, img2])

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for mat in matches:

        # Get the matching keypoints for each of the images
        img1_idx = mat.queryIdx
        img2_idx = mat.trainIdx

        # x - columns
        # y - rows
        (x1,y1) = kp1[img1_idx].pt
        (x2,y2) = kp2[img2_idx].pt

        # Draw a small circle at both co-ordinates
        # radius 4
        # colour blue
        # thickness = 1
        cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
        cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

        # Draw a line in between the two points
        # thickness = 1
        # colour blue
        cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 1)
    cv2.rectangle(out, (int(avg_x + cols1 - length),int(avg_y - length)),(int(avg_x + cols1 + length),int(avg_y + length)), (0, 255, 0), 1)
    
    # Show the image
    cv2.imshow('Matched Features', out)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
image = cv2.imread('sample3.jpg')
scene_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
logo = cv2.imread('FIRSTLogo.jpg', 0)

start = time.time()

sift = cv2.SIFT()

kp1, des1 = sift.detectAndCompute(logo, None)
kp2, des2 = sift.detectAndCompute(scene_image, None)

bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck = True)

matches = bf.match(des1, des2)

matches = sorted(matches, key = lambda x:x.distance)

good_matches = matches[:30]

# Iterate through all matches, and see if the point in surrounded by other
# points in a circle around it
best_matches = []
minRad = 500
minCloseMatches = 5

for match1 in matches:
    numCloseMatches = 0
    x1, y1 = kp2[match1.trainIdx].pt
    for match2 in matches:
        x2, y2 = kp2[match2.trainIdx].pt
        rSquare = (x2 - x1)**2 + (y2 - y1)**2
        if match1 is not match2 and rSquare < minRad:
            numCloseMatches += 1
    if(numCloseMatches > minCloseMatches):
        best_matches.append(match1)

# Re-average with best matches
sum_x = 0
sum_y = 0
for match in best_matches :
    x, y = kp2[match.trainIdx].pt
    sum_x += x
    sum_y += y
avg_x = sum_x/len(best_matches)
avg_y = sum_y/len(best_matches)

# Find new average distance
sum_dist = 0
distances = [None] * len(best_matches)
for i in range(len(best_matches)) :
    x, y = kp2[best_matches[i].trainIdx].pt
    distances[i] = (x - avg_x)**2 + (y - avg_y)**2
    sum_dist += distances[i]
avg_dist = sum_dist/len(matches)

# Calculate the dimensions of the bounding rectangle
length = 4*(avg_dist**0.5)

# Remove everything outside of this region of interest
size = scene_image.shape
roi = np.zeros((size[0], size[1]), dtype = 'uint8')
for i in range(size[1]):
    for j in range(size[0]):
        if i > (avg_x - length) and i < (avg_x + length) and j > (avg_y - length) and j < (avg_y + length) :
            roi[j][i] = scene_image[j][i]

#create the image points
image_points = np.zeros(((len(best_matches)), 2), np.float32)
for i in range(len(best_matches)):
    image_points[i][0], image_points[i][1] = kp2[best_matches[i].trainIdx].pt

# create the object points
IMAGE_WIDTH = logo.shape[1]
IMAGE_HEIGHT = logo.shape[0]
LOGO_WIDTH = 3.0
LOGO_HEIGHT = 3.0 + (5.0/8.0)

object_points = np.zeros(((len(best_matches)), 3), np.float32)
for i in range(len(best_matches)):
    object_points[i][0], object_points[i][1] = kp1[best_matches[i].queryIdx].pt
    object_points[i][0] = (object_points[i][0]/IMAGE_WIDTH)*LOGO_WIDTH
    object_points[i][1] = (object_points[i][1]/IMAGE_HEIGHT)*LOGO_HEIGHT
    object_points[i][2] = 0.0

#set camera matrix
camera_matrix = np.array([[596.01281738, 0, 326.04810079],
                          [0, 589.50994873, 247.26001282],
                          [0.0,  0.0,       1.0]], np.float32)

# set distortion coefficients
dist = np.array([-0.46737483, 0.26953156, -0.00093982, -0.0086415, 0.26339206], np.float32)


# we're here! solvePnP
retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist)

rot_matrix = cv2.Rodrigues(rvec)[0]

pose = -np.matrix(rot_matrix).T * np.matrix(tvec)
print pose

cv2.imshow("RoI", roi)
drawMatches(logo, kp1, scene_image, kp2, best_matches, avg_x, avg_y, int(length))
