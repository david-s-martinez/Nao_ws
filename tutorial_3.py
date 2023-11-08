#!/usr/bin/env python
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

#Top Camera
def callback(data):

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    cv2.imshow('Top camera',cv_image)
    cv2.waitKey(1)
    
    #Convert from BGR to HSV 
    cv_hsvimage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    #Thresholding 

    # Hue : angle of the color on the wheel
    # Saturation : purity of the color
    # Value : Intensity / Brightness

    #Red Color

    #Choosing Upper and lower threshold for Red Color
    lower_red = np.array([0, 100,100], np.uint8)
    upper_red = np.array([10, 255, 255], np.uint8)

    mask_red = cv2.inRange(cv_hsvimage, lower_red, upper_red)


    #Green Color

    #Choosing Upper and lower threshold for Green Color
    lower_green = np.array([40, 40, 40], np.uint8)
    upper_green = np.array([90, 255, 255], np.uint8)

    mask_green = cv2.inRange(cv_hsvimage, lower_green, upper_green)
  
    #Blue Color

    #Choosing Upper and lower threshold for Blue Color
    lower_blue = np.array([100, 50, 50], np.uint8)
    upper_blue = np.array([150, 255, 255], np.uint8)

    mask_blue = cv2.inRange(cv_hsvimage, lower_blue, upper_blue)


    #Show the Image 
    #Make sure to change to the mask needed before runnning 
    cv2.imshow('Color extraction',mask_green)
    cv2.waitKey(1)


    #Blob Extraction

    #Generate Blobs: Dilation adn Erosion 
    kernel = np.ones((5,5), np.uint8)
    

    #Erosion
    cv_erodeimage = cv2.erode(mask_green, kernel, iterations = 1)

    #Dilation
    cv_dilateimage = cv2.dilate(cv_erodeimage, kernel, iterations = 1)




    #Blob Detector 
    # Set up the detector with default parameters.
    params = cv2.SimpleBlobDetector_Params()

    #Set the Parameters Correctly
    # Filter by area (value for area here defines the pixel value)
    params.filterByArea = True
    params.minArea = 100
    
    # Filter by circularity
    params.filterByCircularity = True
    params.minCircularity = 0.75
    
    # Filter by convexity
    params.filterByConvexity = True
    params.minConvexity = 0.2
        
    # Filter by inertia ratio
    params.filterByInertia = True
    params.minInertiaRatio = 0.01


    blobs_detector = cv2.SimpleBlobDetector_create(params)
 
    # Detect blobs.
 
    blobs_in_image = blobs_detector.detect(cv_dilateimage)
 
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    image_with_blobs = cv2.drawKeypoints(cv_dilateimage, blobs_in_image, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # Display Image with Blob Detection 
    cv2.imshow("Blob extraction", image_with_blobs)
    cv2.waitKey(1)



#Buttom Camera
def callback_bottom(data):
   
   # Original Image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    cv2.imshow('Bottom camera',cv_image)
    cv2.waitKey(1)

   # Blurred GrayScale Image

    # GayScale Image 
    cv_grayimage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #Blur the Image 
    cv_blurredgrayimage = cv2.GaussianBlur(cv_grayimage, (9, 9), 2)

    # Display Blurred GrayScale Image
    cv2.imshow('Blureed grayscale',cv_blurredgrayimage)
    cv2.waitKey(1)

    # Circle Detection 
    circles_detection = cv2.HoughCircles(cv_blurredgrayimage, cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=20,
        param1=50,
        param2=30,
        minRadius=10,
        maxRadius=200,
    )

    #Draw the Circles and Center

    if circles_detection is not None:

        # Convert the (x, y) coordinates and radius of the circles to integers
        integers_param = np.uint16(np.around(circles_detection))

        for circle in integers_param[0, :]:
            center = (circle[0], circle[1])
            radius = circle[2]

            # Draw the circle and its center in the frame
            cv2.circle(cv_image, center, radius, (0, 255, 0), 2)
            cv2.circle(cv_image, center, 2, (0, 0, 255), 3)

        # Display Image with Detected Circle
        cv2.imshow("Detected Circles", cv_image)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #Callback is for the Top Camera
    #Callback_bottom is for the Bottom Camera
    
    #rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, callback)

    rospy.Subscriber("/nao_robot/camera/bottom/camera/image_raw", Image, callback_bottom)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
