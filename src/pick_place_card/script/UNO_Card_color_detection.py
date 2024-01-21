#!/usr/bin/env python
import rospy
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

# Callback function for the top camera
def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    
    # Convert from BGR to HSV 
    cv_hsvimage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define color ranges for each color
    color_ranges = {
        'Red': [(0, 100, 100), (10, 255, 255), (160, 100, 100), (180, 255, 255)],
        'Green': [(40, 40, 40), (90, 255, 255)],
        'Yellow': [(20, 100, 100), (40, 255, 255)],
        'Blue': [(90, 50, 50), (120, 255, 255)]
    }

    # Initialize an empty list to store all ellipses
    all_ellipses = []

    for color, ranges in color_ranges.items():
        # Create masks for each color
        masks = [cv2.inRange(cv_hsvimage, np.array(lower), np.array(upper)) for lower, upper in ranges]

        # Combine the masks
        combined_mask = np.sum(masks, axis=0)

        # Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
