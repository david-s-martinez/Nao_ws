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
from nao_control_tutorial_2.srv import CartesianPositionOrientation
from nao_control_tutorial_2.srv import MoveJoints
from nao_control_tutorial_2.srv import TrackAruco
import tf2_ros
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import TransformStamped # Handles TransformStamped message
from sensor_msgs.msg import Image # Image is the message type
from tf2_ros import TransformBroadcaster
import math
#motionProxy =0;

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


        # ROS Initialization
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.reference_frame = 'CameraBottom_optical_frame'
        #self.reference_frame = 'torso'

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

    def broadcast_marker_transform(self, rvec, tvec):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.reference_frame
        transform.child_frame_id = "aruco_marker"  # Adjust the child frame ID as needed

        transform.transform.translation.x = tvec[0]/100
        transform.transform.translation.y = tvec[1]/100
        transform.transform.translation.z = tvec[2]/100

        # aruco_array = np.array(tvec[0]/100,tvec[1]/100,tvec[2]/100, 1)
        # rotation_matrix = cv2.Rodrigues(rvec)[0]
        # quat = tf.transformations.quaternion_from_matrix(rotation_matrix)
        # transform.transform.rotation.x = quat[0]
        # transform.transform.rotation.y = quat[1]
        # transform.transform.rotation.z = quat[2]
        # transform.transform.rotation.w = quat[3]
        transform.transform.rotation.x = 0
        transform.transform.rotation.y = 0
        transform.transform.rotation.z = 0
        transform.transform.rotation.w = 0

        self.tf_broadcaster.sendTransform((tvec[0]/100,tvec[1]/100,tvec[2]/100 ), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "aruco_marker", self.reference_frame)






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
                # self.broadcast_marker_transform(rvec, tvec, tag_id)
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.camera_distortion, rvec, tvec,5)
                # self.draw_tag_pose(frame, rvec, tvec, tag_id)
            return frame, self.rot_vecs[0][0],self.tran_vecs[0][0]

        return frame, [],[]
    
    def run_ros_node(self):
        # Run the ROS node to handle TF broadcasting
        rate = rospy.Rate(10)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            # Perform any ROS-related operations here if needed
            rate.sleep()

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
        
def rotate_matrix(matrix, axis, angle_deg):
    """
    Rotate a given matrix around a specified axis (X, Y, or Z) by a given angle in degrees.

    :param matrix: The matrix to be rotated.
    :param axis: The axis of rotation ('x', 'y', or 'z').
    :param angle_deg: The rotation angle in degrees.
    :return: The rotated matrix.
    """
    angle_rad = math.radians(angle_deg)

    if axis.lower() == 'x':
        R = np.array([
            [1, 0, 0],
            [0, math.cos(angle_rad), -math.sin(angle_rad)],
            [0, math.sin(angle_rad), math.cos(angle_rad)]
        ])
    elif axis.lower() == 'y':
        R = np.array([
            [math.cos(angle_rad), 0, math.sin(angle_rad)],
            [0, 1, 0],
            [-math.sin(angle_rad), 0, math.cos(angle_rad)]
        ])
    elif axis.lower() == 'z':
        R = np.array([
            [math.cos(angle_rad), -math.sin(angle_rad), 0],
            [math.sin(angle_rad), math.cos(angle_rad), 0],
            [0, 0, 1]
        ])
    else:
        raise ValueError("Invalid axis. Choose 'x', 'y', or 'z'.")

    # Extending matrix if necessary to match dimensions
    if matrix.shape[0] == 3 and len(matrix.shape) == 1:  # For 3D vectors
        extended_matrix, jacobian = cv2.Rodrigues(matrix)
        rotated_matrix = np.matmul(R, extended_matrix[:3])
    elif matrix.shape == (3, 3):  # For 3x3 matrices
        rotated_matrix = np.matmul(R, matrix)
    else:
        raise ValueError("Matrix dimensions not supported.")

    return rotated_matrix

class JointControl(object):
    def __init__(self, robotIP = "10.152.246.180", PORT = 9559):
        self.motionProxy = ALProxy("ALMotion", robotIP, PORT)
        call_enable_body_stiffness_service()
        self.move_joints_server()

    #Exercise 1 - 2
    def move_LArm_home(self, req):
        try:

            JointName = 'LArm'
            space = motion.FRAME_TORSO   # Task space {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}.
            useSensorValues = True  #If true, the sensor values will be used to determine the position.
            Position6D = self.motionProxy.getPosition(JointName, space, useSensorValues)
            return (Position6D,)
        except Exception as e:
            print(e)
            print("Move joints failed")
            return False
        
    def do_transform(self, req):
        try:
            original_matrix = np.array([0.0, 0.0, 0.0])  # Example 3D vector
            # Can be 'x', 'y', or 'z'
            angle = 90  # Rotation angle in degrees

            rotated_matrix = rotate_matrix(original_matrix, 'z', angle)
            final_matrix = rotate_matrix(rotated_matrix, 'x', angle)
            camera_bottom2torso = self.HomogeneousTransformation(name  = "CameraBottom")
            camera_bottomOF2torso = np.matmul(final_matrix, camera_bottom2torso)
            print(camera_bottomOF2torso)
            JointName = 'LArm'
            space = motion.FRAME_TORSO   # Task space {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}.
            useSensorValues = True  #If true, the sensor values will be used to determine the position.
            Position6D = self.motionProxy.getPosition(JointName, space, useSensorValues)
            return (Position6D,)
        except Exception as e:
            print(e)
            print("Move joints failed")
            return False
        
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
        
            
    #Exercise 2 - 2 (TBD)
    def ManualTransformation(self):
        pass
                
    # Exercise 2 - 3 
    def HomogeneousTransformation(self, name):
        
        frame  = motion.FRAME_TORSO
        useSensorValues  = True
        result = self.motionProxy.getTransform(name, frame, useSensorValues)
        # for i in range(0, 4):
        #     for j in range(0, 4):
        #         print(result[4*i + j], ',')
        #     print('')
        # rosrun tf tf_echo CameraBottom_optical_frame CameraBottom_frame
        # print(result)
        return np.array(result).reshape((4,4))


    def ArucotoTorsoTransform(self):
        name  = 'aru'
        frame  = motion.FRAME_TORSO
        useSensorValues  = True
        result = self.motionProxy.getTransform(name, frame, useSensorValues)
     # Callback function Aruco with set Angles 
    def get_frame(self,data):
        aruco_det = ArucoDetection()
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        frame, rvec, tvec = aruco_det.detect_tags_3D(cv_image)
        
        if len(tvec)>0:
            aruco_flip = np.eye(4)
            aruco_flip[2,2] = aruco_flip[2,2]*-1
            aruco_flip[:3,-1] = np.transpose(tvec)

            original_matrix = np.array([0.0, 0.0, 0.0])  # Example 3D vector
            angle = -90  # Rotation angle in degrees
            rotated_matrix = rotate_matrix(original_matrix, 'x', angle)
            sensor_corr = rotate_matrix(rotated_matrix, 'z', angle)
            R_corr = np.eye(4)

            R_corr[:3, :3] = sensor_corr
            aruco_corrected = np.matmul(R_corr, aruco_flip)

            angle = 90  # Rotation angle in degrees

            # Can be 'x', 'y', or 'z'
            rotated_matrix = rotate_matrix(original_matrix, 'z', angle)
            final_matrix = rotate_matrix(rotated_matrix, 'x', angle)
            R = np.eye(4)
            R[:3, :3] = final_matrix

            camera_bottom2torso = self.HomogeneousTransformation(name  = "CameraBottom")
            camera_bottomOF2torso = np.matmul(R, camera_bottom2torso)

            tvec_col = aruco_corrected[:,-1]
            aruco2torso = np.matmul(camera_bottomOF2torso,tvec_col)
            aruco_det.broadcast_marker_transform(rvec, aruco2torso)
            # aruco_det.broadcast_marker_transform(rvec, tvec)
        print(len(tvec),tvec)

        cv2.imshow('Original Image',cv_image)
        cv2.imshow('Track Image',frame)
        key = cv2.waitKey(1)

    def track_aruco(self, req):
        try:
            start = req.start
            if start == True:
                rospy.Subscriber("/nao_robot/camera/bottom/camera/image_raw", Image, self.get_frame)
                rospy.spin()
            return True
            
        except Exception as e:
            print(e)
            print("Move joints failed")
            return False
    def move_joints_server(self):
        rospy.init_node('move_joints_server')
        service1 = rospy.Service('joint_cartesian_CordOr', CartesianPositionOrientation, self.joint_cartesian_coordinates)
        service2 = rospy.Service('joint_cartesian_CordOr_movement', MoveJoints, self.joint_cartesian_coordinates_displacement)
        service3 = rospy.Service('move_LArm_home', CartesianPositionOrientation, self.move_LArm_home)
        service4 = rospy.Service('track_aruco', TrackAruco, self.track_aruco)
        service5 = rospy.Service('do_transform', CartesianPositionOrientation, self.do_transform)
        print("Ready to move joints.")
        rospy.spin()

# 	('Rotated Matrix:', array([[  6.12323400e-17,   0.00000000e+00,   1.00000000e+00],
#    [  1.00000000e+00,   6.12323400e-17,  -6.12323400e-17],
#    [ -6.12323400e-17,   1.00000000e+00,   3.74939946e-33]]))

if __name__ == '__main__':
    # motionProxy = ALProxy("ALMotion", robotIP, PORT)
    # rospy.init_node('move_joints_server')
    # rospy.spin()
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    # print sys.argv[2]
 
    jc = JointControl(robotIP,PORT)