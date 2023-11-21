#!/usr/bin/env python
import rospy
import time
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_1.srv import MoveJoints
motionProxy = 0
from std_srvs.srv import Empty  # Import the appropriate service message type

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

#TODO: create service handler
class JointControl(object):
    def __init__(self, robotIP = "10.152.246.180", PORT = 9559):
        self.motionProxy = ALProxy("ALMotion", robotIP, PORT)
        call_enable_body_stiffness_service()
        self.move_joints_server()

    def handle_move_joints(self, req):
        try:
            time.sleep(1.0)
            # self.motionProxy.setStiffnesses("Head", 1.0)
            name = req.name
            angle = req.angle*almath.TO_RAD
            fractionMaxSpeed = req.speed
            self.motionProxy.setAngles(name, angle, fractionMaxSpeed)
            print("Moving joint: " + name)
            time.sleep(3.0)
            # self.motionProxy.setStiffnesses("Head", 0.0)
            time.sleep(1.0)
            return True
        except Exception as e:
            print(e)
            print("Move joints failed")
            return False

    def move_joints_server(self):
        rospy.init_node('move_joints_server')
        s = rospy.Service('move_joints_tutorial', MoveJoints, self.handle_move_joints)
        print("Ready to move joints.")
        rospy.spin()

if __name__ == '__main__':
    # motionProxy = ALProxy("ALMotion", robotIP, PORT)
    # rospy.init_node('move_joints_server')
    # #TODO init service
    # rospy.spin()
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    jc = JointControl(robotIP,PORT)