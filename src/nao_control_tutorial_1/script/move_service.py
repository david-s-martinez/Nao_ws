#!/usr/bin/env python
import rospy
import time
import almath
import sys
from naoqi import ALProxy
from nao_control_tutorial_1.srv import MoveJoints
from nao_control_tutorial_1.srv import InterpolationMultiJoints
from nao_control_tutorial_1.srv import InterpolationJoints
from nao_control_tutorial_1.srv import TrackAruco
motionProxy = 0
from std_srvs.srv import Empty  # Import the appropriate service message type
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoDetection:
    def __init__(self, marker_size= 8.7,tag_scaling = 0.5, box_z = 3.0, tag_dict = cv2.aruco.DICT_ARUCO_ORIGINAL):
        """
        ArucoDetection object constructor. Initializes data containers.
        camera matrix
        502.63849959,   0. ,        328.96515157
        0.    ,     502.63849959, 249.74604858
        0.    ,       0.      ,     1. 
        (8.75345261e+00,5.07315184e+01,8.25609725e-03,3.04573657e-03,2.10942303e+02,8.75507174e+00,4.83657869e+01,2.10477474e+02,0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00)

        """
        self.box_z = box_z
        self.id_to_find  = 0
        self.marker_size  = marker_size #cm
        self.tag_scaling = tag_scaling
        self.homography = None
        self.Rot_x = np.array([
                    [1.0, 0.0, 0.0],
                    [0.0, math.cos(math.radians(180)),-math.sin(math.radians(180))],
                    [0.0, math.sin(math.radians(180)), math.cos(math.radians(180))]])
        
        self.tag_boxes = {}
        self.box_vertices = {}
        self.plane_world_pts = {}
        self.plane_world_pts_detect = []
        self.plane_img_pts_detect = []

        self.camera_matrix = np.array(
            [[502.63849959,   0. ,        328.96515157],
            [0.    ,     502.63849959, 249.74604858],
            [ 0.    ,       0.      ,     1.] ])
        
        self.camera_distortion = np.array(
            [8.75345261e+00,5.07315184e+01,8.25609725e-03,3.04573657e-03,2.10942303e+02,8.75507174e+00,4.83657869e+01,2.10477474e+02,0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00,0.00000000e+00])
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(tag_dict)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def draw_tag_pose(self,image, rvec, tvec, tag_id, z_rot=-1):
        world_points = np.array([
            0, 0, 0,
            4, 0, 0,
            0, 4, 0,
            0, 0, -4 * z_rot,
            1,1,0
        ]).reshape(-1, 1, 3) * self.tag_scaling * self.marker_size

        img_points, _ = cv2.projectPoints(world_points, 
                                            rvec, tvec, 
                                            self.camera_matrix, 
                                            self.camera_distortion)
        img_points = np.round(img_points).astype(int)
        img_points = [tuple(pt) for pt in img_points.reshape(-1, 2)]

        cv2.line(image, img_points[0], img_points[1], (0,0,255), 2)
        cv2.line(image, img_points[0], img_points[2], (0,255,0), 2)
        cv2.line(image, img_points[0], img_points[3], (255,69,0), 2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, 'X', img_points[1], font, 0.5, (0,0,255), 2, cv2.LINE_AA)
        cv2.putText(image, 'Y', img_points[2], font, 0.5, (0,255,0), 2, cv2.LINE_AA)
        cv2.putText(image, 'Z', img_points[3], font, 0.5, (255,69,0), 2, cv2.LINE_AA)
        cv2.putText(image, str((tag_id)), (img_points[4][0],img_points[4][1]), font, 0.5,
                                        (255,255,0), 2, cv2.LINE_AA)
        
    def detect_tags_3D(self, frame):
        self.box_vertices = {}
        self.box_verts_update = {}
        self.rot_vecs = []
        self.tran_vecs = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        corners, ids, rejected = cv2.aruco.detectMarkers(
                                            gray, 
                                            self.aruco_dict, 
                                            parameters = self.parameters,
                                            cameraMatrix = self.camera_matrix, 
                                            distCoeff = self.camera_distortion)

        # if ids is not None and (self.id_to_find in ids):
        if ids is not None :
            poses = cv2.aruco.estimatePoseSingleMarkers(
                                                corners, 
                                                self.marker_size*self.tag_scaling, 
                                                self.camera_matrix, 
                                                self.camera_distortion)

            min_id = min(ids[0])
            self.rot_vecs, self.tran_vecs = poses[0], poses[1]
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i, tag_id in enumerate(ids):
                
                rvec , tvec = self.rot_vecs[i][0], self.tran_vecs[i][0]
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.camera_distortion, rvec, tvec,5)
                # self.draw_tag_pose(frame, rvec, tvec, tag_id)

        return frame, self.tran_vecs, corners

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
            # self.motionProxy.setStiffnesses("Head", 1.0)
            name = req.name
            angle = req.angle*almath.TO_RAD
            fractionMaxSpeed = req.speed
            self.motionProxy.setAngles(name, angle, fractionMaxSpeed)
            print("Moving joint: " + name)
            time.sleep(0.1)
            # self.motionProxy.setStiffnesses("Head", 0.0)
            return True
        except Exception as e:
            print(e)
            print("Move joints failed")
            return False
    # Callback function Aruco with set Angles 
    def get_frame(self,data):
        aruco_det = ArucoDetection()
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        frame, tran_vecs, corners = aruco_det.detect_tags_3D(cv_image)
        print(len(corners),corners)
        if len(tran_vecs) > 0:
            tran_vec = tran_vecs[0][0]
            tran_vec = [tran_vec[0],tran_vec[1], tran_vec[2]]
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

            # Calculate the center of the image 
            # image_width, image_height  = frame.shape[:2]
            # image_xcenter = image_width / 2.0 
            # image_ycenter = image_height / 2.0

            #Based on NAO documentation 
            image_xcenter = 640 / 2.0
            image_ycenter = 480 / 2.0


            #Calculate the displacement between the centroid of the ArUco and the center of the image
            dx =  image_xcenter - cX
            dy = cY - image_ycenter 


            # Find the angle using the horizontal and vertical fov mapping (Nao's documentation)

            yaw_angle = (dx/640 ) * 60.9
            pitch_angle = (dy / 480 ) * 47.6
            names = [ 'HeadYaw', 'HeadPitch']
            angles = [yaw_angle*almath.TO_RAD, pitch_angle*almath.TO_RAD]
            fractionMaxSpeed = 0.1
            self.motionProxy.setAngles(names, angles, fractionMaxSpeed)
            
            print(tran_vec)
        #cv2.rectangle(cv_image, (0, 0), (100, 100), color=(255,0,0), thickness=2)
        cv2.imshow('Original Image',cv_image)
        cv2.imshow('Track Image',frame)
        key = cv2.waitKey(1)

    # Callback function Aruco with set Angles 
    def get_frame_interpolation(self,data):
        aruco_det = ArucoDetection()
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        frame, tran_vecs, corners = aruco_det.detect_tags_3D(cv_image)
        print(len(corners),corners)
        if len(tran_vecs) > 0:
            tran_vec = tran_vecs[0][0]
            tran_vec = [tran_vec[0],tran_vec[1], tran_vec[2]]
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

            # Calculate the center of the image 
            # image_width, image_height  = frame.shape[:2]
            # image_xcenter = image_width / 2.0 
            # image_ycenter = image_height / 2.0

            #Based on NAO documentation 
            image_xcenter = 640 / 2.0
            image_ycenter = 480 / 2.0


            #Calculate the displacement between the centroid of the ArUco and the center of the image
            dx =  image_xcenter - cX
            dy = cY - image_ycenter 


            # Find the angle using the horizontal and vertical fov mapping (Nao's documentation)
            yaw_angle = (dx/640 ) * 60.9
            pitch_angle = (dy / 480 ) * 47.6
            names = ['HeadYaw', 'HeadPitch']
            angles = [yaw_angle*almath.TO_RAD, pitch_angle*almath.TO_RAD]
            isAbsolute = True
            maxSpeedFraction = 0.2
            timeLists  = 2.0


            self.motionProxy.angleInterpolationWithSpeed(names, angles, maxSpeedFraction)

            # Call getTaskList to have the previous angleInterpolation task number
            taskList = self.motionProxy.getTaskList()

            self.motionProxy.killTask(taskList[0][1])

            print(tran_vec)
        #cv2.rectangle(cv_image, (0, 0), (100, 100), color=(255,0,0), thickness=2)
        cv2.imshow('Original Image',cv_image)
        cv2.imshow('Track Image',frame)
        key = cv2.waitKey(1)

    def track_aruco(self, req):
        try:
            start = req.start
            if start == True:
                rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.get_frame)
                rospy.spin()
            elif start == False:
                rospy.Subscriber("/nao_robot/camera/top/camera/image_raw", Image, self.get_frame_interpolation)
                rospy.spin()
            return True
        except Exception as e:
            print(e)
            print("Move joints failed")
            return False
        
    def handle_interpolation_joints(self, req):
        try:
            time.sleep(1.0)
            # self.motionProxy.setStiffnesses("Head", 1.0)
            joint_names = req.joint_names
            anglesList = [angle*almath.TO_RAD for angle in req.anglesList]
            time_assigned = req.time_assigned
            isAbsolute =req.isAbsolute
            self.motionProxy.angleInterpolation(joint_names, anglesList, time_assigned, isAbsolute)
            print("Moving joint: " + str(joint_names))
            time.sleep(3.0)
            # self.motionProxy.setStiffnesses("Head", 0.0)
            time.sleep(1.0)
            return True
        except Exception as e:
            print(e)
            print("Move joints failed")
            return False
    def handle_interpolation_multi_joints(self, req):
        try:
            time.sleep(1.0)
            # self.motionProxy.setStiffnesses("Head", 1.0)
            joint_names = req.joint_names
            anglesList = [angle*almath.TO_RAD for angle in req.anglesList]
            time_assigned = list(req.time_assigned)
            isAbsolute =req.isAbsolute
            # Transform it into arrays of arrays based on the siye of joint names 
            # anglesList = np.array(anglesList)
            # anglesList = anglesList.reshape((-1,5))
            # print(anglesList)

            #time also should be transformed
            # time_assigned = np.array(time_assigned)
            # time_assigned = time_assigned.reshape((-1,5))
            # print(time_assigned)

            # Reshape into a list of lists
            angles_list_formatted = []
            time_assigned_formatted = []

            chunk_size = 1  # Define the number of elements per inner list

            # Create sublists of 'chunk_size' elements for anglesList
            for i in range(0, len(anglesList), chunk_size):
                angles_list_formatted.append(anglesList[i:i + chunk_size])

            # Create sublists of 'chunk_size' elements for time_assigned
            for i in range(0, len(time_assigned), chunk_size):
                time_assigned_formatted.append(time_assigned[i:i + chunk_size])

            print(angles_list_formatted)
            print(time_assigned_formatted)


            self.motionProxy.angleInterpolation(joint_names, angles_list_formatted, time_assigned_formatted, isAbsolute)
            print("Moving joint: " + str(joint_names))
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
        service1 = rospy.Service('move_joints_tutorial', MoveJoints, self.handle_move_joints)
        service2 = rospy.Service('move_joints_tutorial_interpolation', InterpolationJoints, self.handle_interpolation_joints)
        service3 = rospy.Service('move_multijoints_tutorial_interpolation', InterpolationMultiJoints, self.handle_interpolation_multi_joints)
        service4 = rospy.Service('track_aruco', TrackAruco, self.track_aruco)
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