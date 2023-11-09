#!/usr/bin/env python
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def callback(data):

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    #cv2.rectangle(cv_image, (0, 0), (100, 100), color=(255,0,0), thickness=2)
    cv2.imshow('Original Image',cv_image)
    key = cv2.waitKey(1)
    #Press s to capture an image 
    if key == ord('s'):
        cv2.imwrite('templateImg.jpg', cv_image)
    if key == ord('r'):
        #Read the image
        saved_image = cv2.imread('templateImg.jpg')
        
        cv2.imshow('Processed Image',saved_image)
        cv2.selectROI(saved_image)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
