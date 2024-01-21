#!/usr/bin/env python
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


# Function to find the major color in a bounding box
def find_major_color(mask_red, mask_green, mask_yellow, mask_blue):
    total_pixels = mask_red.size

    count_red = cv2.countNonZero(mask_red)
    count_green = cv2.countNonZero(mask_green)
    count_yellow = cv2.countNonZero(mask_yellow)
    count_blue = cv2.countNonZero(mask_blue)

    # Find the color with the maximum count
    max_count = max(count_red, count_green, count_yellow, count_blue)

    # Check if there is a major color and its percentage is above a threshold
    if max_count / total_pixels > 0.5:
        if max_count == count_red:
            return "Red"
        elif max_count == count_green:
            return "Green"
        elif max_count == count_yellow:
            return "Yellow"
        elif max_count == count_blue:
            return "Blue"

    return None 

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
    lower_red1 = np.array([0, 100,100], np.uint8)
    upper_red1 = np.array([10, 255, 255], np.uint8)

    lower_red2 = np.array([160, 100,100], np.uint8)
    upper_red2 = np.array([10, 255, 255], np.uint8)

    # Create masks for both red ranges
    mask_red1 = cv2.inRange(cv_hsvimage, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(cv_hsvimage, lower_red2, upper_red2)

    # Combine the masks
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)


    #Green Color

    #Choosing Upper and lower threshold for Green Color
    lower_green = np.array([40, 40, 40], np.uint8)
    upper_green = np.array([90, 255, 255], np.uint8)

    mask_green = cv2.inRange(cv_hsvimage, lower_green, upper_green)
  
    #Blue Color

    #Choosing Upper and lower threshold for Blue Color
    lower_blue = np.array([90, 50, 50], np.uint8)
    upper_blue = np.array([150, 255, 255], np.uint8)

    mask_blue = cv2.inRange(cv_hsvimage, lower_blue, upper_blue)


    #Choosing Upper and lower threshold for Yellow Color
    lower_yellow = np.array([20, 100, 100], np.uint8)
    upper_yellow = np.array([40, 255, 255], np.uint8)

    mask_yellow = cv2.inRange(cv_hsvimage, lower_yellow, upper_yellow)


    # Combine the masks
    combined_mask = np.sum((mask_red1, mask_red2), axis=0)

    # Find contours
    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    all_ellipses = []
    # Draw ellipses around each contour
    for contour in contours:
        # Fit an ellipse around the contour
        ellipse = cv2.fitEllipse(contour)
        all_ellipses.append(ellipse)

    # Draw all ellipses on the frame
    for ellipse in all_ellipses:
        cv2.ellipse(cv_image, ellipse, (0, 255, 0), 2)

    # Display the frame with ellipses
    cv2.imshow('Ellipse Detection', cv_image)

    # Break the loop when 'q' key is pressed
    cv2.waitKey(1)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #Callback is for the Top Camera
    #Callback_bottom is for the Bottom Camera
    
    rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, callback)

    #rospy.Subscriber("/nao_robot/camera/bottom/camera/image_raw", Image, callback_bottom)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()