#!/usr/bin/env python


# Team C Group Members:
# Efe Oztufan
# David Sebastian Martinez Lema
# Jorje Fabrizio Villasante Meza
# Maria Luna Ghanime


import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
#import matplotlib.pyplot as plt
class ArucoDetection:
    def __init__(self, marker_size= 4,tag_scaling = 1, box_z = 3.0, tag_dict = cv2.aruco.DICT_ARUCO_ORIGINAL):
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
                                                self.marker_size, 
                                                self.camera_matrix, 
                                                self.camera_distortion)

            min_id = min(ids[0])
            self.rot_vecs, self.tran_vecs = poses[0], poses[1]
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i, tag_id in enumerate(ids):
                
                rvec , tvec = self.rot_vecs[i][0], self.tran_vecs[i][0]
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.camera_distortion, rvec, tvec,5)
                # self.draw_tag_pose(frame, rvec, tvec, tag_id)
        return frame

def backprojection(cv_image, hist_normalized, mask):
    hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # back_projection = cv2.calcBackProject([hsv_frame], [0,1], hist_normalized, [0, 180, 0, 256], 1)
    back_projection = cv2.calcBackProject([hsv_frame], [0], hist_normalized, [0, 180], 1)
    disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    cv2.filter2D(back_projection,-1,disc,back_projection)
    back_proj_filt = cv2.bitwise_and(back_projection, back_projection, mask=mask)
    thresh = cv2.merge((back_proj_filt,back_proj_filt,back_proj_filt))
    res = cv2.bitwise_and(cv_image, thresh)
    tres = np.hstack((thresh,res))
    return res, tres, back_proj_filt

backp = False
backm = False
backc = False
backo = False



def callback(data):
    global backp, backm, backc, backo
    ad = ArucoDetection()
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    #cv2.rectangle(cv_image, (0, 0), (100, 100), color=(255,0,0), thickness=2)
    cv2.imshow('Original Image',cv_image)
    key = cv2.waitKey(1)
    aruco_img = ad.detect_tags_3D(cv_image.copy())
    cv2.imshow('3D Marker Position',aruco_img)
    key = cv2.waitKey(1)
    #Task 5: HSV, Threshold, Mask 
    hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    h, s, _ = cv2.split(hsv_frame)
    roi = (155, 86, 54, 57)      # Modify accordingly


    #Press s to capFalseture an image 
    if key == ord('s'):
        cv2.imwrite('templateImg.jpg', cv_image)
        # saved_image = cv2.imread('templateImg.jpg')
    if key == ord('r'):
        #Read the image
        saved_image = cv2.imread('templateImg.jpg')
        print("Image shape:", saved_image.shape)
        
        cv2.imshow('Processed Image',saved_image)
        cv2.waitKey(1)

        roi_cropped = saved_image[int(roi[1]):int(roi[1] + roi[3]), int(roi[0]):int(roi[0] + roi[2])]
        roi_hsv = cv2.cvtColor(roi_cropped, cv2.COLOR_BGR2HSV)
        roi_h, roi_s, _ = cv2.split(roi_hsv)
        
        thresh = 125  # Modify if needed
        _, roi_mask = cv2.threshold(roi_s, thresh, 255, cv2.THRESH_BINARY)

        cv2.imshow("ROI",roi_mask)

        # Display the image 
        cv2.imshow('Region of Interest', roi_cropped)
        cv2.waitKey(1)

        # Make a Screenshot of it 
        cv2.imwrite('roi_screenshot.jpg', roi_cropped)


    #Task 4
    if key == ord('t'):
        #Read the image
        roi_image = cv2.imread('roi_screenshot.jpg')
        print("Image shape:", roi_image.shape)

        #Display Image
        cv2.imshow('ROI Image',roi_image)
        cv2.waitKey(1)

        #ROI to HSV 
        roi_hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
                
        cv2.imshow('HSV Image',roi_hsv)
        cv2.waitKey(1)

        #Split the Image
        roi_h, roi_s, _ = cv2.split(roi_hsv)


        #Threshold to mask out the lref_gray, p0ow saturated pixels
        threshold = 120  
        _, roi_mask = cv2.threshold(roi_s, threshold, 255, cv2.THRESH_BINARY)
        cv2.imshow("ROI Mask",roi_mask)
        cv2.waitKey(1)


        #Compute the 1D Histogram and Normalize it 

        roi_hist = cv2.calcHist([roi_h],[0], roi_mask, [180], [0, 180] )
        r_hist_norm = roi_hist.copy()
        cv2.normalize(roi_hist,r_hist_norm,0,255,cv2.NORM_MINMAX)
        np.save("norm_histo.npy", r_hist_norm)

        # plt.figure((12,6))

        # plt.subplot(1,2,1)
        # plt.plot(roi_hist)
        # plt.title('ROI Histogram')

        # plt.subplot(1,2,2)
        # plt.plot(roi_hist)
        # plt.title('ROI Normalized Histogram')
    
    thresh = 110  # Modify if needed
    _, mask = cv2.threshold(s, thresh, 255, cv2.THRESH_BINARY)
    # tres = norm_hist(cv_image,mask)

    # Task 5                # Press b to see the backprojection and b again to stop seeing it
    if key == ord('b'):     # BackProject
        backp = not backp
        cv2.destroyAllWindows()


    if backp:
        # norm_hist( cv_image, mask)
        hist_normalized = np.load("norm_histo.npy")
        _, tres, _ = backprojection(cv_image, hist_normalized, mask)
        cv2.imshow("Back Projection", tres)
        cv2.waitKey(1)

# def norm_hist(cv_image,mask ):
    
    # Task 6                # Press m to see the backprojection and m again to stop seeing it
    if key == ord('m'):     # MeanShift
        backm = not backm
        cv2.destroyAllWindows()


    if backm:
        hist_normalized = np.load("norm_histo.npy")
        res, _, back_proj_filt = backprojection(cv_image, hist_normalized, mask)

        # Define the initial tracking window (rectangle from the ROI selection)
        track_window = tuple(map(int, roi))

        _, track_window = cv2.meanShift(back_proj_filt, track_window, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1))
        x, y, w, h = track_window
        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        tres = np.hstack((cv_image,res))

        cv2.imshow("Mean-shift Back Projection", tres)
        # cv2.destroyAllWindows()

    # Task 7                # Press c to see the backprojection and c again to stop seeing it
    if key == ord('c'):     # CamShift
        backc = not backc
        cv2.destroyAllWindows()


    if backc:
        hist_normalized = np.load("norm_histo.npy")
        res, _, back_proj_filt = backprojection(cv_image, hist_normalized, mask)

        # Define the initial tracking window (rectangle from the ROI selection)
        track_window = tuple(map(int, roi))

        ret, track_window = cv2.CamShift(back_proj_filt, track_window, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1))
        pts = cv2.boxPoints(ret)
        pts = np.int0(pts)
        frame = cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)    # Update of cv_image

        tres = np.hstack((frame,res))

        cv2.imshow("Back Projection", tres)
        # cv2.destroyAllWindows()

    # Task 9                # Press o to see the OpticalFlow and o again to stop seeing it
    if key == ord('o'):     # OpticalFlow
        backo = not backo
        cv2.destroyAllWindows()
    
    if backo:
        # Create a mask image for drawing purposes
        mask = np.zeros_like(cv_image)

        try:
            p0 = np.load("p0.npy")
            ref_gray = cv2.imread('ref_gray.jpg')
            ref_gray = cv2.cvtColor(ref_gray, cv2.COLOR_BGR2GRAY)

        except:
            ref_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            p0 = cv2.goodFeaturesToTrack(ref_gray, maxCorners=100, qualityLevel=0.3, minDistance=7)
            # np.save("p0.npy", p0)
            # cv2.imwrite('ref_gray.jpg', frame_gray)

        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # print(ref_gray.shape)
        # print(frame_gray.shape)

         # Calculate optical flow using Lucas-Kanade method
        p1, st, _ = cv2.calcOpticalFlowPyrLK(ref_gray, frame_gray, p0, None)
        good_new = p1[st == 1]
        good_old = p0[st == 1]

        # Draw the traces of motion
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
            frame = cv2.circle(cv_image, (int(a), int(b)), 5, (0, 0, 255), -1)

        img = cv2.add(frame, mask)
        cv2.imshow("Optical Flow", img)

        # Update the reference frame and points for the next iteration
        cv2.imwrite('ref_gray.jpg', frame_gray)
        p0 = good_new.reshape(-1, 1, 2)
        np.save("p0.npy", p0)
        # cv2.destroyAllWindows()


        
        






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
