#!/usr/bin/env python
import rospy
import time
import motion
import numpy as np
import almath
import sys
from naoqi import ALProxy
from std_srvs.srv import Empty  # Import the appropriate service message type
import cv2
import tf
# from pick_place_card.srv import Pose2D
import tf2_ros
from pick_place_card.srv import CartesianPositionOrientation
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import TransformStamped # Handles TransformStamped message
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image # Image is the message type
from tf2_ros import TransformBroadcaster
import math
#motionProxy =0;

def call_enable_body_stiffness_service():
    rospy.wait_for_service('/body_stiffness/enable')  # Replace with the actual service name
    try:
        # Create a proxy to call the service
        enable_body_stiffness_proxy = rospy.ServiceProxy('/body_stiffness/enable', Empty)
        
        # Call the service with an empty request
        response = enable_body_stiffness_proxy()
        
        # Process the response (empty for this service)
        print("Enabled")
    except rospy.ServiceException as e:
        print("Service cansform.transform.translation.x = tvec[0][0]ll failed:", str(e))

def call_disable_body_stiffness_service():
    rospy.wait_for_service('/body_stiffness/disable')  # Replace with the actual service name
    try:
        # Create a proxy to call the service
        disable_body_stiffness_proxy = rospy.ServiceProxy('/body_stiffness/disable', Empty)
        
        # Call the service with an empty request
        response = disable_body_stiffness_proxy()
        
        # Process the response (empty for this service)
        print("Disabled")
    except rospy.ServiceException as e:
        print("Service call failed:", str(e))
        
class MovetoTarget(object):

    def __init__(self, robotIP = "10.152.246.180", PORT = 9559):

        self.motionProxy = ALProxy("ALMotion", robotIP, PORT)
        self.postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
        call_enable_body_stiffness_service()
        self.pub = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        
        self.move_joints_server()

    def get_LArm_positions(self, req):
            try:
             #LArm Rest Position Cartesian_Position_Orientation: [0.020364301279187202, 0.12601032853126526, -0.11523474752902985, -0.6465103626251221, 1.423244833946228, 0.6563076972961426]
             #Bottom Part Cartesian_Position_Orientation: [0.15335305035114288, 0.06843108683824539, -0.01828966662287712, -1.910516619682312, 0.23121587932109833, -0.029442522674798965]
             #Upper Part Cartesian_Position_Orientation: [0.19423720240592957, 0.11404170095920563, 0.02963467501103878, -1.4287537336349487, 0.0009230562718585134, 0.13425925374031067]
                

            # UP Cartesian_Position_Orientation: [0.20340755581855774, 0.15785562992095947, 0.07319368422031403, -1.6913267374038696, -0.09684331715106964, 0.355697363615036]
            # DOWN [0.1751420795917511, 0.1285971701145172, 0.0202433243393898, -2.150057554244995, 0.01597822830080986, 0.21731269359588623]
            # MID Cartesian_Position_Orientation: [0.121991366147995, 0.11651371419429779, 0.007444058544933796, -2.1198246479034424, -0.05146960914134979, 0.1333589255809784]
                JointName = 'LArm'
                space = motion.FRAME_TORSO   # Task space {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}.
                useSensorValues = True  #If true, the sensor values will be used to determine the position.
                Position6D = self.motionProxy.getPosition(JointName, space, useSensorValues)
                return (Position6D,)
            except Exception as e:
                print(e)
                print("Move joints failed")
                return False
            
    def LArm_init(self):
        try: 
            
            Position6DLeft = [0.121991366147995, 0.11651371419429779, 0.007444058544933796, -2.1198246479034424, -0.05146960914134979, 0.1333589255809784]
            JointName = "LArm"
            space = motion.FRAME_TORSO
            MaximumVelocity = 0.9
            self.motionProxy.setPosition(JointName, space, Position6DLeft, MaximumVelocity, 63)
            print('Move to the mid position')

        
        except Exception as e:
                print(e)
                print("Move joints failed")
                return False
        

    def move_LArm(self):
        try:

            JointName = "LArm"
            space = motion.FRAME_TORSO
            MaximumVelocity = 0.4
            Position6DLeft_Down =  [0.1751420795917511, 0.1285971701145172, 0.0202433243393898, -2.150057554244995, 0.01597822830080986, 0.21731269359588623]
            self.motionProxy.setPosition(JointName, space, Position6DLeft_Down, MaximumVelocity, 63)
            print('Move Down Row')
            rospy.sleep(2)
            Position6DLeft_MoveBack = [0.121991366147995, 0.11651371419429779, 0.007444058544933796, -2.1198246479034424, -0.05146960914134979, 0.1333589255809784]
            self.motionProxy.setPosition(JointName, space, Position6DLeft_MoveBack, MaximumVelocity, 63)
            print('Move Back with Card')
            # Position6DLeft_Up = [0.20340755581855774, 0.15785562992095947, 0.07319368422031403, -1.6913267374038696, -0.09684331715106964, 0.355697363615036]
            # self.motionProxy.setPosition(JointName, space, Position6DLeft_Up, MaximumVelocity, 63)
            # print('Move Upper Row')


        except Exception as e:
                print(e)
                print("Move joints failed")
                return False
        
        
    def move_to_Object(self):
        try:
            
            #Middle Low Picked 
            pose_msg = Pose2D()
            pose_msg.x = 0.13
            pose_msg.y = 0
            pose_msg.theta = 0.0
            leftArmEnable  = False
            rightArmEnable = True
            self.motionProxy.setMoveArmsEnabled(leftArmEnable, rightArmEnable)
            self.pub.publish(pose_msg)
            print('tried to walk')


            #Left Low Picked
            # pose_msg = Pose2D()
            # pose_msg.x = 0.12
            # pose_msg.y = 0.05
            # pose_msg.theta = 0.0
            # self.postureProxy.goToPosture("StandInit", 0.5)
            # rospy.sleep(2)
            # self.pub.publish(pose_msg)
            # print('tried to walk')
            

            return True

        except Exception as e:
            print(e)
            print("Move to Target failed")
            return False
        
    def move_back(self):
        try:
            
            #Middle Low Picked 
            pose_msg = Pose2D()
            pose_msg.x = 0.14
            pose_msg.y = 0
            pose_msg.theta = 0.0
            self.pub.publish(pose_msg)
            rospy.sleep(2)
            self.postureProxy.goToPosture("StandZero", 1)

            
            print('tried to walk')


            #Left Low Picked
            # pose_msg = Pose2D()
            # pose_msg.x = 0.12
            # pose_msg.y = 0.05
            # pose_msg.theta = 0.0
            # self.postureProxy.goToPosture("StandInit", 0.5)
            # rospy.sleep(2)
            # self.pub.publish(pose_msg)
            # print('tried to walk')
            

            return True

        except Exception as e:
            print(e)
            print("Move to Target failed")
            return False
        
        
    def throw_card(self):
        try:
            
            #Middle Low Picked 
            pose_msg = Pose2D()
            pose_msg.x = -0.01
            pose_msg.y = -0.12
            pose_msg.theta = 0.0
            leftArmEnable  = False
            rightArmEnable = True
            self.motionProxy.setMoveArmsEnabled(leftArmEnable, rightArmEnable)
            self.pub.publish(pose_msg)
            print('tried to walk')
            rospy.sleep(2)
            self.postureProxy.goToPosture("StandInit", 1)
            # JointName = "LArm"
            # space = motion.FRAME_TORSO
            # MaximumVelocity = 0.9
            # Position6DLeft_PutCard =  [0.12016086280345917, 0.039947617799043655, -0.029062306508421898, -2.256195306777954, 0.29694950580596924, -0.2803468704223633]
            # self.motionProxy.setPosition(JointName, space, Position6DLeft_PutCard, MaximumVelocity, 63)
            # print('Move Down Row')



            #Left Low Picked
            # pose_msg = Pose2D()
            # pose_msg.x = 0.12
            # pose_msg.y = 0.05
            # pose_msg.theta = 0.0
            # self.postureProxy.goToPosture("StandInit", 0.5)
            # rospy.sleep(2)
            # self.pub.publish(pose_msg)
            # print('tried to walk')
            

            return True

        except Exception as e:
            print(e)
            print("Move to Target failed")
            return False
        
        
    def move_joints_server(self):
        rospy.init_node('move_joints_server')
        service3 = rospy.Service('get_LArm_positions', CartesianPositionOrientation, self.get_LArm_positions)
        # service1 = rospy.Service('FeetMovement', Pose2D, self.move_to_Object)
        print("Ready to move joints.")
        self.LArm_init()
        rospy.sleep(2)
        self.move_to_Object()
        rospy.sleep(2)
        self.move_LArm()
        rospy.sleep(2)
        self.throw_card()

   
        #self.throw_card()
        # self.move_back()
        #rospy.sleep(3)
        # self.move_back()
        rospy.spin()
    
# 	('Rotated Matrix:', array([[  6.12323400e-17,   0.00000000e+00,   1.00000000e+00],
#    [  1.00000000e+00,   6.12323400e-17,  -6.12323400e-17],
#    [ -6.12323400e-17,   1.00000000e+00,   3.74939946e-33]]))

if __name__ == '__main__':
    # motionProxy = ALProxy("ALMotion", robotIP, PORT)
    # rospy.init_node('move_joints_server')
    # rospy.spin()
    # robotIP=str(sys.argv[1])
    # PORT=int(sys.argv[2])
    # print sys.argv[2]
 
    jc = MovetoTarget()