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
from Aruco_Marker import ArucoDetection
from Aruco_Marker import rotate_matrix
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
        
class NAOMove(object):

    def __init__(self, robotIP = "10.152.246.180", PORT = 9559, needs_node=True):

        self.motionProxy = ALProxy("ALMotion", robotIP, PORT)
        self.postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
        call_enable_body_stiffness_service()
        self.pub = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.current_state = 1
        self.move_joints_server(needs_node)

    def nao_walk(self,x ,y, theta):
        pose_msg = Pose2D()
        pose_msg.x = x
        pose_msg.y = y
        pose_msg.theta = theta
        self.pub.publish(pose_msg)
        print('tried to walk')

    def get_LArm_positions(self, req):
            try:
             #LArm Rest Position Cartesian_Position_Orientation: [0.020364301279187202, 0.12601032853126526, -0.11523474752902985, -0.6465103626251221, 1.423244833946228, 0.6563076972961426]
             #Bottom Part Cartesian_Position_Orientation: [0.15335305035114288, 0.06843108683824539, -0.01828966662287712, -1.910516619682312, 0.23121587932109833, -0.029442522674798965]
             #Upper Part Cartesian_Position_Orientation: [0.19423720240592957, 0.11404170095920563, 0.02963467501103878, -1.4287537336349487, 0.0009230562718585134, 0.13425925374031067]
                

            # UP Cartesian_Position_Orientation: [0.20340755581855774, 0.15785562992095947, 0.07319368422031403, -1.6913267374038696, -0.09684331715106964, 0.355697363615036]
            # DOWN [0.1751420795917511, 0.1285971701145172, 0.0202433243393898, -2.150057554244995, 0.01597822830080986, 0.21731269359588623]
            # MID Cartesian_Position_Orientation: [0.121991366147995, 0.11651371419429779, 0.007444058544933796, -2.1198246479034424, -0.05146960914134979, 0.1333589255809784]
                JointName = req.JointName
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
            
            #Position6DLeft = [0.121991366147995, 0.11651371419429779, 0.007444058544933796, -2.1198246479034424, -0.05146960914134979, 0.1333589255809784]
            Position6DLeft = [0.11926589906215668, 0.08077982068061829, -0.04658479616045952, -0.16600564122200012, 0.2785385251045227, 0.02825542539358139]
            JointName = "LArm"
            space = motion.FRAME_TORSO
            MaximumVelocity = 0.9
            self.motionProxy.setPosition(JointName, space, Position6DLeft, MaximumVelocity, 63)
            print('Move to the mid position')

        
        except Exception as e:
                print(e)
                print("Move joints failed")
                return False
        
    
    def RArm_init(self):
        try: 
            
            #Position6DLeft = [0.121991366147995, 0.11651371419429779, 0.007444058544933796, -2.1198246479034424, -0.05146960914134979, 0.1333589255809784]
            Position6DRight =  [0.13970205187797546, -0.10936237871646881, 0.008094454184174538, 1.9241509437561035, -0.05772218480706215, -0.16212432086467743]
            JointName = "RArm"
            space = motion.FRAME_TORSO
            MaximumVelocity = 0.9
            self.motionProxy.setPosition(JointName, space, Position6DRight, MaximumVelocity, 63)
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
            # Position6DLeft_Down =  [0.1751420795917511, 0.1285971701145172, 0.0202433243393898, -2.150057554244995, 0.01597822830080986, 0.21731269359588623]
            # self.motionProxy.setPosition(JointName, space, Position6DLeft_Down, MaximumVelocity, 63)
            # print('Move Down Row')
            # rospy.sleep(2)
            # Position6DLeft_MoveBack = [0.121991366147995, 0.11651371419429779, 0.007444058544933796, -2.1198246479034424, -0.05146960914134979, 0.1333589255809784]
            # self.motionProxy.setPosition(JointName, space, Position6DLeft_MoveBack, MaximumVelocity, 63)
            # print('Move Back with Card')
            # Position6DLeft_Up = [0.20340755581855774, 0.15785562992095947, 0.07319368422031403, -1.6913267374038696, -0.09684331715106964, 0.355697363615036]
            # self.motionProxy.setPosition(JointName, space, Position6DLeft_Up, MaximumVelocity, 63)
            # print('Move Upper Row')
            Position6D_Up = [0.18167956173419952, 0.10307253897190094, -0.01296320278197527, -0.2035345882177353, 0.22035706043243408, 0.09165364503860474]
            self.motionProxy.setPosition(JointName, space, Position6D_Up, MaximumVelocity, 63)
            print('Choose Up Middle')

        except Exception as e:
                print(e)
                print("Move joints failed")
                return False
        
    def move_RArm_withHead(self):
        try:

            JointName = "RArm"
            #JointNameT = "LArm"
            space = motion.FRAME_TORSO
            MaximumVelocity = 0.4
            # Position6DLeft_Down =  [0.1751420795917511, 0.1285971701145172, 0.0202433243393898, -2.150057554244995, 0.01597822830080986, 0.21731269359588623]
            # self.motionProxy.setPosition(JointName, space, Position6DLeft_Down, MaximumVelocity, 63)
            # print('Move Down Row')
            # rospy.sleep(2)
            # Position6DLeft_MoveBack = [0.121991366147995, 0.11651371419429779, 0.007444058544933796, -2.1198246479034424, -0.05146960914134979, 0.1333589255809784]
            # self.motionProxy.setPosition(JointName, space, Position6DLeft_MoveBack, MaximumVelocity, 63)
            # print('Move Back with Card')
            # Position6DLeft_Up = [0.20340755581855774, 0.15785562992095947, 0.07319368422031403, -1.6913267374038696, -0.09684331715106964, 0.355697363615036]
            # self.motionProxy.setPosition(JointName, space, Position6DLeft_Up, MaximumVelocity, 63)
            # print('Move Upper Row')
            # Position6D_RightArm = [0.05069751664996147, 0.12137755006551743, -0.10618214309215546, -1.0323925018310547, 1.1300336122512817, 0.03199877589941025]
            # self.motionProxy.setPosition(JointNameT, space, Position6D_RightArm, MaximumVelocity, 63)
            # print('Move Hand Up')
            leftArmEnable  = True
            rightArmEnable = False
            self.motionProxy.setMoveArmsEnabled(leftArmEnable, rightArmEnable)
            # self.nao_walk(x = 0.025,y = 0.002, theta = 0.0)
            # rospy.sleep(2)
            Position6D_MoveUp = [0.13642016053199768, -0.20635484158992767, 0.10498404502868652, 0.6817927360534668, -0.27265381813049316, -0.12625375390052795]
            self.motionProxy.setPosition(JointName, space, Position6D_MoveUp, MaximumVelocity, 63)
            print('Move Hand Up')
            rospy.sleep(2)
            Position6D_MoveUp2 =[0.21727561950683594, -0.11308850347995758, 0.07528077065944672, 0.007104993797838688, 0.05709807947278023, 0.018436921760439873]
            self.motionProxy.setPosition(JointName, space, Position6D_MoveUp2, MaximumVelocity, 63)
            print('Move Hand Up 2')
            rospy.sleep(2)
            Position6D_GrabCard = [0.18486575782299042, -0.13559098541736603, -0.011911261826753616, -0.012399778701364994, 0.5004745721817017, -0.12544162571430206]
            self.motionProxy.setPosition(JointName, space, Position6D_GrabCard, 0.9, 63)
            print('Grab Card')
            rospy.sleep(2)
            Position_Trial =  [0.21372699737548828, -0.11542697995901108, 0.1451791226863861, 0.082606740295887, -0.2623349130153656, 0.0008494996000081301]
            self.motionProxy.setPosition(JointName, space, Position_Trial, 0.9, 63)
            print('Move Hand Up')
            rospy.sleep(1)
            Position6D_Look = [0.10344336926937103, -0.13088473677635193, 0.23385189473628998, 1.8361880779266357, -1.346941590309143, 0.46041110157966614]
            self.motionProxy.setPosition(JointName, space, Position6D_Look, MaximumVelocity, 63)
            print('Look at the card')
            rospy.sleep(2)
            JointNameH = 'Head'
            Position6D_Head = [0.0, 0.0, 0.1264999955892563, 0.0, -0.5538160800933838, -0.8836259841918945]
            self.motionProxy.setPosition(JointNameH, space, Position6D_Head, MaximumVelocity, 63)
            print('Head Rotation')
            rospy.sleep(2)  
            self.RArm_init()
            Position6D_Head = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.motionProxy.setPosition(JointNameH, space, Position6D_Head, MaximumVelocity, 63)
            print('Head Rotation Back')
            rospy.sleep(2)

        except Exception as e:
            print(e)
            print("Move joints failed")
            return False

    def move_RArm_withLeftArm(self):
            try:

                JointName = "RArm"
                JointNameT = "LArm"
                space = motion.FRAME_TORSO
                MaximumVelocity = 0.4
                # Position6DLeft_Down =  [0.1751420795917511, 0.1285971701145172, 0.0202433243393898, -2.150057554244995, 0.01597822830080986, 0.21731269359588623]
                # self.motionProxy.setPosition(JointName, space, Position6DLeft_Down, MaximumVelocity, 63)
                # print('Move Down Row')
                # rospy.sleep(2)
                # Position6DLeft_MoveBack = [0.121991366147995, 0.11651371419429779, 0.007444058544933796, -2.1198246479034424, -0.05146960914134979, 0.1333589255809784]
                # self.motionProxy.setPosition(JointName, space, Position6DLeft_MoveBack, MaximumVelocity, 63)
                # print('Move Back with Card')
                # Position6DLeft_Up = [0.20340755581855774, 0.15785562992095947, 0.07319368422031403, -1.6913267374038696, -0.09684331715106964, 0.355697363615036]
                # self.motionProxy.setPosition(JointName, space, Position6DLeft_Up, MaximumVelocity, 63)
                # print('Move Upper Row')
                Position6D_RightArm = [0.05069751664996147, 0.12137755006551743, -0.10618214309215546, -1.0323925018310547, 1.1300336122512817, 0.03199877589941025]
                self.motionProxy.setPosition(JointNameT, space, Position6D_RightArm, MaximumVelocity, 63)
                print('Move Hand Up')
                rospy.sleep(2)
                
                leftArmEnable  = True
                rightArmEnable = False
                #self.motionProxy.setMoveArmsEnabled(leftArmEnable, rightArmEnable)
                # self.nao_walk(x = 0.025,y = 0.002, theta = 0.0)
                rospy.sleep(2)
                Position6D_MoveUp = [0.13642016053199768, -0.20635484158992767, 0.10498404502868652, 0.6817927360534668, -0.27265381813049316, -0.12625375390052795]
                self.motionProxy.setPosition(JointName, space, Position6D_MoveUp, MaximumVelocity, 63)
                print('Move Hand Up')
                rospy.sleep(2)
                Position6D_MoveUp2 = [0.21799898147583008, -0.1084299087524414, 0.07960008829832077, -0.009287073276937008, 0.037209682166576385, 0.04144500941038132]
                self.motionProxy.setPosition(JointName, space, Position6D_MoveUp2, MaximumVelocity, 63)
                print('Move Hand Up 2')
                rospy.sleep(2)
                Position6D_GrabCard = [0.18486575782299042, -0.13559098541736603, -0.011911261826753616, -0.012399778701364994, 0.5004745721817017, -0.12544162571430206]
                self.motionProxy.setPosition(JointName, space, Position6D_GrabCard, 0.9, 63)
                print('Grab Card')
                rospy.sleep(2)
                Position_Trial =  [0.21372699737548828, -0.11542697995901108, 0.1451791226863861, 0.082606740295887, -0.2623349130153656, 0.0008494996000081301]
                self.motionProxy.setPosition(JointName, space, Position_Trial, 0.9, 63)
                print('Move Hand Up')
                rospy.sleep(2)
                Position6D_PoseCard = [0.21205469965934753, -0.04830517619848251, 0.09707397222518921, 0.13737207651138306, -0.03729049488902092, 0.35286054015159607]
                self.motionProxy.setPosition(JointName, space, Position6D_PoseCard, MaximumVelocity, 63)
                print('Intermediate Pose Card')
                rospy.sleep(2)
                Position6D_PoseCard2 =  [0.18808002769947052, -0.03795960545539856, 0.005345016717910767, -0.018800245597958565, 0.3773193955421448, 0.4288717806339264]
                self.motionProxy.setPosition(JointName, space, Position6D_PoseCard2, MaximumVelocity, 63)
                print('Pose Card')
                # rospy.sleep(5)
                # Position6D_EndMovement = [0.08185836672782898, -0.04597493261098862, -0.0026721833273768425, 1.7218643426895142, 0.1917998343706131, 1.0842033624649048]
                # self.motionProxy.setPosition(JointName, space, Position6D_EndMovement, MaximumVelocity, 63)
                # print('Pose Card')



            except Exception as e:
                    print(e)
                    print("Move joints failed")
                    return False

    def move_head_Start(self):
        #Head Motion at the beginning of the game 
        try:
            JointNameH = 'Head'
            space = motion.FRAME_TORSO
            MaximumVelocity = 0.4
            Position6D_Head = [0.0, 0.0, 0.1264999955892563, 0.0, 0.43561410903930664, 0.33743810653686523]
            self.motionProxy.setPosition(JointNameH, space, Position6D_Head, MaximumVelocity, 63)
            print('Head Rotation')
            rospy.sleep(2)  
            Position6D_Head = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.motionProxy.setPosition(JointNameH, space, Position6D_Head, MaximumVelocity, 63)  
        except Exception as e:
            print(e)
            print("Head Movement failed")
            return False   
        
    def move_to_Object(self):
        try:
            #Middle Low Picked 
            leftArmEnable  = False
            rightArmEnable = False
            self.motionProxy.setMoveArmsEnabled(leftArmEnable, rightArmEnable)
            self.nao_walk(x = 0.13,y = 0.0, theta = 0.0)
            return True

        except Exception as e:
            print(e)
            print("Move to Target failed")
            return False
        
    def move_back(self, ):
        try:
            
            #Middle Low Picked 
            self.nao_walk(x = 0.14,y = 0.0, theta = 0.0)
            rospy.sleep(2)
            self.postureProxy.goToPosture("StandZero", 1)
            return True

        except Exception as e:
            print(e)
            print("Move to Target failed")
            return False
        
    def move_yaxis(self, cX, cY):
        try: 
            # if cX < 265 and cY < 45:
            #     self.nao_walk(x= 0.0, y = -0.01, theta = 0.0)
            #     rospy.sleep(1)
            #     self.nao_walk(x= 0.01, y = 0.0, theta = 0.0)
            #     rospy.sleep(1)
            #     self.postureProxy.goToPosture("StandInit", 1)
            #     corrected_y = False

            # elif cX > 275 and cY > 55:
            #     self.nao_walk(x= 0.0, y = 0.01, theta = 0.0)
            #     rospy.sleep(1)
            #     self.nao_walk(x= -0.01, y = 0.0, theta = 0.0)
            #     rospy.sleep(1)
            #     self.postureProxy.goToPosture("StandInit", 1)
            #     corrected_y = False

            # elif cX > 275 and cY < 45:
            #     self.nao_walk(x= 0.0, y = -0.01, theta = 0.0)
            #     rospy.sleep(1)
            #     self.nao_walk(x= -0.01, y = 0.0, theta = 0.0)
            #     rospy.sleep(1)
            #     self.postureProxy.goToPosture("StandInit", 1)
            #     corrected_y = False   

            # elif cX < 265 and cY > 55:
            #     self.nao_walk(x= 0.0, y = 0.01, theta = 0.0)
            #     rospy.sleep(1)
            #     self.nao_walk(x= 0.01, y = 0.0, theta = 0.0)
            #     rospy.sleep(1)
            #     self.postureProxy.goToPosture("StandInit", 1)
            #     corrected_y = False 

            # elif cX < 265:
            #     self.nao_walk(x= 0.0, y = 0.01, theta = 0.0)
            #     rospy.sleep(1)
            #     self.postureProxy.goToPosture("StandInit", 1)
            # elif cX > 275:
            #     self.nao_walk(x= 0.0, y = -0.01, theta = 0.0)
            #     rospy.sleep(1)
            #     self.postureProxy.goToPosture("StandInit", 1)
            # elif cY > 55:
            #     self.nao_walk(x= -0.01, y = 0.0, theta = 0.0)
            #     rospy.sleep(1)
            #     self.postureProxy.goToPosture("StandInit", 1)
            #     corrected_y = False
            # elif cY < 45:
            #     self.nao_walk(x= 0.01, y = 0.0, theta = 0.0)
            #     rospy.sleep(1)
            #     self.postureProxy.goToPosture("StandInit", 1)
            #     corrected_y = False
            # else: 
            #     self.postureProxy.goToPosture("StandInit", 1)
            #     corrected_y = True
            
            # return corrected_y

            #Based on NAO documentation 
            image_xcenter = 640 / 2.0
            image_ycenter = 480 / 2.0


            #Calculate the displacement between the centroid of the ArUco and the center of the image
            dx =  image_xcenter - cX
            dy = cY - image_ycenter 
            print(dx,dy)

            # Find the angle using the horizontal and vertical fov mapping (Nao's documentation)

            yaw_angle = (dx/640 ) * 60.9
            pitch_angle = (dy / 480 ) * 47.6
            names = [ 'HeadYaw', 'HeadPitch']
            angles = [yaw_angle*almath.TO_RAD, pitch_angle*almath.TO_RAD]
            fractionMaxSpeed = 0.1
            self.motionProxy.setAngles(names, angles, fractionMaxSpeed)

            

        except Exception as e:
            print(e)
            print("Move to Target failed")
            return False

    def move_xaxis(self, cY):
        try: 
            if cY < 45:
                self.nao_walk(x= 0.01, y = 0.0 , theta = 0.0)
                corrected_x = False

            elif cY > 55:
                self.nao_walk(x= -0.01, y = 0.0 , theta = 0.0)
                corrected_x = False
            else: 
                self.postureProxy.goToPosture("StandInit", 1)
                corrected_x = True
            
            return corrected_x

        except Exception as e:
            print(e)
            print("Move to Target failed")
            return False
        
        
    def throw_card(self):
        try:
            
            #Middle Low Picked 
            leftArmEnable  = False
            rightArmEnable = True
            self.motionProxy.setMoveArmsEnabled(leftArmEnable, rightArmEnable)
            self.nao_walk(x = -0.01, y = -0.12, theta = 0.0)
            rospy.sleep(2)
            return True

        except Exception as e:
            print(e)
            print("Move to Target failed")
            return False
        
    def put_card(self):
        try:
            JointName = "LArm"
            space = motion.FRAME_TORSO
            MaximumVelocity = 0.6
            Position6DLeft_PutCard = [0.1629522293806076, 0.17891249060630798, 0.08092391490936279, -0.014790819957852364, -0.06110773980617523, -0.10336251556873322]
            self.motionProxy.setPosition(JointName, space, Position6DLeft_PutCard, MaximumVelocity, 63)
            rospy.sleep(2)
            Position6DLeft_PutCard =[0.20895427465438843, 0.05273515731096268, 0.05187284201383591, -0.52846360206604, 0.1518012136220932, -0.261351615190506]
            self.motionProxy.setPosition(JointName, space, Position6DLeft_PutCard, MaximumVelocity, 63)
            print('Movement 2')


            return True

        except Exception as e:
            print(e) 
            print("Move to Target failed")
            return False   
        

    def Left_Hand_Movement(self):
        # pick a card on shelf with left tool
        try:
            self.LArm_init()
            rospy.sleep(2)
            #self.move_to_Object()
            # rospy.sleep(2)
            self.move_LArm()
            rospy.sleep(2)
            self.LArm_init()
            rospy.sleep(2)
            self.throw_card()
            rospy.sleep(2)
            self.move_LArm()
            rospy.sleep(2)
            self.LArm_init()
            rospy.sleep(2)
            self.postureProxy.goToPosture("StandInit", 0.5)
            rospy.sleep(2)
            self.nao_walk(x = 0.0, y = 0.13, theta = 0.0)

        except Exception as e:
            print(e) 
            print("Left Hand Movement Fail")
            return False   
        
    
    def Right_Hand_MovementHead(self):
        # pick card from the deck, then look at it.
        try:
            self.RArm_init()
            rospy.sleep(2)
            self.move_RArm_withHead()


        
        except Exception as e:
            print(e) 
            print("Left Hand Movement Fail")
            return False   
        
    def Right_Hand_MovementLeft(self):
        # pick card from the deck and put it on the table.
        try:
            self.RArm_init()
            rospy.sleep(2)
            self.move_RArm_withLeftArm()
            rospy.sleep(2)
            self.RArm_init()

        
        except Exception as e:
            print(e) 
            print("Left Hand Movement Fail")
            return False   
        

    def HomogeneousTransformation(self, name):
        
        frame  = motion.FRAME_TORSO
        useSensorValues  = True
        result = self.motionProxy.getTransform(name, frame, useSensorValues)
        return np.array(result).reshape((4,4))

    # Callback function Aruco with set Angles 
    def get_frame(self,data):
        aruco_det = ArucoDetection()
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        frame, rvec, tvec, corners = aruco_det.detect_tags_3D(cv_image)

        if len(tvec)>0:
            corner = corners[0][0]
            corner = corner.reshape((4, 2))
            (top_left, top_right, bottom_right, bottom_left) = corner 
            top_left = (int(top_left[0]), int(top_left[1]))
            top_right = (int(top_right[0]), int(top_right[1]))
            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

            # Compute centroid
            cX = int((top_left[0] + bottom_right[0]) / 2.0)
            cY = int((top_left[1] + bottom_right[1]) / 2.0)
            #print(len(corners),cX,cY)
            print('Centroid: ')
            
            # print(cX,cY)
            # print(frame.shape)

            aruco_flip = np.eye(4)
            aruco_flip[2,2] = aruco_flip[2,2]#*-1
            aruco_flip[:3,-1] = np.transpose(tvec)
            original_matrix = np.array([0.0, 0.0, 0.0])  # Example 3D vector
            angle = -90  # Rotation angle in degrees
            rotated_matrix = rotate_matrix(original_matrix, 'x', angle)
            sensor_corr = rotate_matrix(rotated_matrix, 'z', angle)
            R_corr = np.eye(4)

            #R_corr[:3, :3] = sensor_corr
            R_corr[:3, :3] = sensor_corr
            aruco_corrected = np.matmul(R_corr, aruco_flip)
            angle = 90  # Rotation angle in degrees

            # Can be 'x', 'y', or 'z'
            rotated_matrix = rotate_matrix(original_matrix, 'z', angle)
            final_matrix = rotate_matrix(rotated_matrix, 'x', angle)
            R = np.eye(4)
            R[:3, :3] = final_matrix

            camera_bottom2torso = self.HomogeneousTransformation(name  = "CameraBottom")
            camera_bottomOF2torso = np.matmul(camera_bottom2torso, R)

            tvec_col = aruco_corrected[:,-1]
            aruco2torso = np.matmul(camera_bottomOF2torso,tvec_col)
            aruco_det.broadcast_marker_transform(rvec, aruco2torso)
            
            #Based on NAO documentation 
            image_x = 50
            image_y = 269


            #Calculate the displacement between the centroid of the ArUco and the center of the image
            dx =  image_x - cX
            dy = cY - image_y
            # print(dx,dy)

            # Find the angle using the horizontal and vertical fov mapping (Nao's documentation)

            desired_matrix =   [[-0.99969168 , 0.00926312 , 0.02303767 ,-0.34885616],
                                [ 0.02339162 , 0.66255697 , 0.74864618 , 0.1894097 ],
                                [-0.00832897 , 0.74895425 ,-0.66256937 , 6.67519127]]
            
            rmat,_ = cv2.Rodrigues(rvec)
            tmat = np.array(tvec).reshape((3,1))

            current_pose = np.hstack((rmat,tmat))
            # print(current_pose)

            # self.nao_walk(x= dx*0.001, y = dy*0.001, theta = 0.0)
        else:
            #Consecutive Iteration with no aruco model 
            if self.current_state:
                self.counterright = 0
                self.counterleft = 0
                self.current_state = False

            else: 

                self.counterright = self.counterright + 1
                self.counterleft = self.counterleft + 1
                #print(self.counterright)
                if (self.counterright > 20):
                    self.counterright = 0
                    # [0.15583667159080505, -0.11708324402570724, 0.25175711512565613, 1.7100205421447754, -0.8724943995475769, -0.18669183552265167]
                    PositionMatrixRight = [0.15583667159080505, -0.11708324402570724, 0.25175711512565613]
                    OrientationMatrixRight = [1.7100205421447754, -0.8724943995475769, -0.18669183552265167]
                    #Dont mind about the orientation matrix 
                    Position6D = PositionMatrixRight + OrientationMatrixRight
                    JointName = "RArm"
                    space = motion.FRAME_TORSO

                    # # print(Position6D)
                    MaximumVelocity = 0.9
                    # self.motionProxy.setPosition(JointName, space, Position6D, MaximumVelocity, 63)

                if (self.counterleft > 20):
                    self.counterleft = 0
                    # [0.13588638603687286, 0.05417526513338089, 0.25502467155456543, -2.096224546432495, -1.0449066162109375, -0.30391818284988403]
                    PositionMatrixLeft = [0.13588638603687286, 0.05417526513338089, 0.25502467155456543]
                    OrientationMatrixLeft = [-2.096224546432495, -1.0449066162109375, -0.30391818284988403]
                    #Dont mind about the orientation matrix 
                    Position6D = PositionMatrixLeft + OrientationMatrixLeft
                    JointName = "LArm"
                    space = motion.FRAME_TORSO

                    # # print(Position6D)
                    MaximumVelocity = 0.9
                    # self.motionProxy.setPosition(JointName, space, Position6D, MaximumVelocity, 63)

                self.current_state = False
            print(self.counterright)

        cv2.imshow('Original Image',cv_image)
        cv2.imshow('Track Image',frame)
        key = cv2.waitKey(1)


    def track_aruco(self):
        try:
                       
            
            rospy.Subscriber("/nao_robot/camera/bottom/camera/image_raw", Image, self.get_frame)
            cv2.imshow()
            return True
    
        except Exception as e:
                print(e)
                print("Move joints failed")
                return False

    def move_joints_server(self, needs_node):
        if needs_node:
            rospy.init_node('move_joints_server')
        service3 = rospy.Service('get_LArm_positions', CartesianPositionOrientation, self.get_LArm_positions)
        # service1 = rospy.Service('FeetMovement', Pose2D, self.move_to_Object)
        print("Ready to move joints.")
        self.postureProxy.goToPosture("StandInit", 0.5)
        rospy.sleep(2)
        if needs_node:
            mode = raw_input("choose right or left movement, r/l/c/t \n")
            print(mode)
            if mode == 'l':
                self.Left_Hand_Movement()
            elif mode =='r':
                self.Right_Hand_MovementHead()
            elif mode =='c':
                self.Right_Hand_MovementLeft()

            # Test mode to be deleted later 
            elif mode == 't':

                self.track_aruco()


            else:
                print('Error')
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
 
    jc = NAOMove()