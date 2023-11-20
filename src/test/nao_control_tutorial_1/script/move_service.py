#!/usr/bin/env python
import rospy
import time
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_1.srv import MoveJoints
motionProxy = 0
#TODO: create service handler
class JointControl(object):
    def __init__(self, robotIP = "10.152.246.118", PORT = 9559):
        self.motionProxy = ALProxy("ALMotion", robotIP, PORT) 
        self.motionProxy.setStiffnesses("Head", 1.0)
        self.move_joints_server()

    def handle_move_joints(self, req):
        # print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
        try:
            # self.motionProxy.setStiffnesses("Head", 1.0)
            name = req.name
            angle = req.angle*almath.TO_RAD
            fractionMaxSpeed = req.speed
            self.motionProxy.setAngles(name, angle, fractionMaxSpeed)
            print("Moving joint: " + name)
            time.sleep(3.0)
            # self.motionProxy.setStiffnesses("Head", 0.0)
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