#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.image_callback)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
                print(e)


        cv2.imshow("Original Image", cv2_img)

        # Convert to grayscale
        gray_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Greyscale Image", gray_img)

        # Convert to binary image
        _, binary_img = cv2.threshold(gray_img, 127, 255, cv2.THRESH_BINARY)
        cv2.imshow("Binary Image", binary_img)

        cv2.waitKey(1)



def main():
    ic = ImageConverter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
            print("End.")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()