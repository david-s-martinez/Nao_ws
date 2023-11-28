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
from nao_control_tutorial_2.srv import CartesianPositionOrientation
from nao_control_tutorial_2.srv import MoveJoints
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
        print("Service call failed:", str(e))

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

class JointControl(object):
    def __init__(self, robotIP = "10.152.246.180", PORT = 9559):
        self.motionProxy = ALProxy("ALMotion", robotIP, PORT)
        call_enable_body_stiffness_service()
        self.move_joints_server()

    #Exercise 1 - 2
    def joint_cartesian_coordinates(self, req):
        try:
            # self.motionProxy.setStiffnesses("Head", 1.0)
            JointName = req.JointName
            space = motion.FRAME_TORSO   # Task space {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}.
            useSensorValues = True  #If true, the sensor values will be used to determine the position.
            Position6D = self.motionProxy.getPosition(JointName, space, useSensorValues)
            # self.motionProxy.setStiffnesses("Head", 0.0)
            return (Position6D,)
        except Exception as e:
            print(e)
            print("Move joints failed")
            return False
        
    #Exercise 1 - 3
    def joint_cartesian_coordinates_displacement(self, req):
        try:

            # self.motionProxy.setStiffnesses("Head", 1.0)
            JointName = req.JointName
            PositionMatrix = req.PositionMatrix
            OrientationMatrix = req.OrientationMatrix
            Position6D = PositionMatrix + OrientationMatrix
            MaximumVelocity = req.MaximumVelocity
            ExecutionTime = req.ExecutionTime
            space = motion.FRAME_TORSO   # Task space {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}.
            useSensorValues = True  #If true, the sensor values will be used to determine the position.
            # print("Keep the value of the orientation matrix empty if you don#t want to specify them")
            # print("Would you like to specify the desired fraction of maximal velocity or the desired execution time?")
            # desired_mode = print(" Please press V for maximal velocity and T for execution time.")

            # if (desired_mode == 'V'): 
            #     print("You chose to specify the fraction of maximal velocity")

            # axis mask = 7 if User Only specified the position
            if (MaximumVelocity != 0.0 and OrientationMatrix == [0.0,0.0,0.0]):
                self.motionProxy.setPosition(JointName, space, Position6D, MaximumVelocity, 7)
                print("Velocity not equal zero and Orientation Matrix 0 ")
            elif (MaximumVelocity != 0.0 ):
                self.motionProxy.setPosition(JointName, space, Position6D, MaximumVelocity, 63)
                print("Velocity not equal zero")
            # # axis mask = 63 if User specified the position and orientation
            # elif (MaximumVelocity != 0):
                #self.motionProxy.setPosition(JointName, space, Position6D, MaximumVelocity, Mask)

            # elif(desired_mode == 'T'):
            #     print("You chose to specify the execution time")
            
            # axis mask = 7 if User Only specified the position
            elif (ExecutionTime != 0.0 and OrientationMatrix == [0.0,0.0,0.0]):
                self.motionProxy.positionInterpolations(JointName, space, Position6D, 7, ExecutionTime)
                print("Time not equal zero and Orientation Matrix 0 ")
            elif (ExecutionTime != 0.0):
                self.motionProxy.positionInterpolations(JointName, space, Position6D, 63, ExecutionTime)
                print("Time not equal zero")
            # axis mask = 63 if User specified the position and orientation
            # elif (ExecutionTime != 0):
            #     self.motionProxy.positionInterpolations(JointName, space, Position6D, 63, ExecutionTime)

            return (Position6D,)
                
        except Exception as e:
            print(e)
            print("Move joints failed")
            return False



    def move_joints_server(self):
        rospy.init_node('move_joints_server')
        service1 = rospy.Service('joint_cartesian_CordOr', CartesianPositionOrientation, self.joint_cartesian_coordinates)
        service2 = rospy.Service('joint_cartesian_CordOr_movement', MoveJoints, self.joint_cartesian_coordinates_displacement)
        print("Ready to move joints.")
        rospy.spin()
		
if __name__ == '__main__':
    # motionProxy = ALProxy("ALMotion", robotIP, PORT)
    # rospy.init_node('move_joints_server')
    # rospy.spin()
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    # print sys.argv[2]
    jc = JointControl(robotIP,PORT)