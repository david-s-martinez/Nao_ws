#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    
    # Display the original image
    cv2.imshow('Original Image', cv_image)
    
    # Convert the image to grayscale
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv2.imshow('Greyscale Image', gray_image)
    
    # Threshold the grayscale image to create a binary image
    _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
    cv2.imshow('Binary Image', binary_image)

    # Wait for a key press and close the windows if 'q' is pressed
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        cv2.destroyAllWindows()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
