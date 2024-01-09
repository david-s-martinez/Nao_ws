#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <stdlib.h>
#include <ros/ros.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
// #include <aruco/aruco.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include "sensor_msgs/JointState.h"
#include "nao_control_tutorial_2/MoveJoints.h"
#include <string.h>

using namespace std;

class Nao_control
{
public:
    // ros handler
    ros::NodeHandle nh_;

    // Image transport and subscriber:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Mat image;


    // subscriber to joint states
    ros::Subscriber sensor_data_sub;

    // Aruco camera parametersOrientationMatrix
    //aruco::CameraParameters cameraParameters;

    // Aruco marker parameters
    float aruco_x, aruco_y, aruco_z;
    float markerSize;
    

    Nao_control() : it_(nh_)
    {
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 500, &Nao_control::imageCallBack, this);

        sensor_data_sub = nh_.subscribe("/joint_states", 1, &Nao_control::sensorCallback, this);

        // TODO: Initialize the service client
        move_joints_client = nh_.serviceClient<nao_control_tutorial_2::MoveJoints>("joint_cartesian_CordOr_movement");
    }

    ~Nao_control()
    {
    }


    // handler for joint states
    void sensorCallback(const sensor_msgs::JointState::ConstPtr &jointState)
    {
        // TODO: Process joint state data
    }

    // Image callback function is executed when a new image is received:
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg) 
    {
        ROS_INFO_STREAM("Image callback execution");
        cv_bridge::CvImageConstPtr cvImagePointer;
	try 
	{
            cvImagePointer = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
	    image = cvImagePointer->image.clone();
	}
	catch (cv_bridge::Exception& except) 
	{
	    ROS_ERROR("cv_bridge exception: %s", except.what());
            return;
        }

	// Distortion matrix:
        cv::Mat dist = (cv::Mat_<double>(4, 1) <<
                -0.0870160932911717,
                0.128210165050533,
                0.003379500659424,
                -0.00106205540818586);

	// Camera matrix:
        cv::Mat cameraP = (cv::Mat_<double>(3, 3) <<
                274.139508945831, 0.0, 141.184472810944,
                0.0, 275.741846757374, 106.693773654172,
                0.0, 0.0, 1.0);

	// cameraParameters.setParams(cameraP,dist,cv::Size(640,480));
    //     aruco::MarkerDetector arucoDetector;
    //     std::vector<aruco::Marker> arucoMarkers;
    //     arucoDetector.detect(image, arucoMarkers);

    //     if (arucoMarkers.size() > 0) 
	// {
	//     ROS_WARN_STREAM("I could detect "<<arucoMarkers.size()<<" aruco marker(s).");
	//     markerSize = 0.064;
	//     arucoMarkers[0].calculateExtrinsics(markerSize, cameraParameters, true);
    //         arucoMarkers[0].draw(image, cv::Scalar(0,0,255), 2);
    //         aruco_x = arucoMarkers[0].Tvec.at<float>(0);
    //         aruco_y = arucoMarkers[0].Tvec.at<float>(1);
    //         aruco_z = arucoMarkers[0].Tvec.at<float>(2);
    //     }
	// else
	// {
	//     ROS_WARN_STREAM("No aruco marker detected");
    //         aruco_x = 0.0;
    //         aruco_y = 0.0;
    //         aruco_z = 0.0;
	// }
	// ROS_INFO_STREAM("aruco_x ="<<aruco_x);
	// ROS_INFO_STREAM("aruco_y ="<<aruco_y);
	// ROS_INFO_STREAM("aruco_z ="<<aruco_z);

	// // Display marker
    //     cv::imshow("marker", image);
    //     cv::waitKey(3);
    }

    bool callMoveJointsService(const std::string &JointName, double PositionMatrix[3], double OrientationMatrix[3], double MaximumVelocity, double ExecutionTime)
    {
        // Create the service message
        nao_control_tutorial_2::MoveJoints srv;
        srv.request.JointName = JointName;
        for (size_t i = 0; i < 3; ++i)
        {
            srv.request.PositionMatrix[i] = PositionMatrix[i];
            srv.request.OrientationMatrix[i] = OrientationMatrix[i];
        }

        srv.request.MaximumVelocity = MaximumVelocity;
        srv.request.ExecutionTime = ExecutionTime;


        // Call the service
        move_joints_client.call(srv);
    }

// TODO: create function for each task
private: 
    ros::ServiceClient move_joints_client;


};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nao_tutorial_control_2");
    Nao_control my_NAO_ControlInstance;

    while (true)
    {
        char x; 
        char y;
        cout << "\nThis code helps you move an end effector of your choice of the NAO robot. Before doing so, few questions should be asked \n";
        cout << " Would you like to specify the desired cartesian position only or both the cartesian position and orientation? \n";
        cout << " Press Y for Position and Orientation and N for Position Only. \n";
        cin >> x; // Get user input from the keyboard
        cout << " Now that you have specified the desired mode would you want to specify the fraction of maximum speed or the total execution time \n";
        cout << " Press V for Velocity Specification and T for Execution Time Specification. \n";
        cin >> y;
        std::string joint_name ;
        cout << "Please enter the desired joint name: ";
        cin >> joint_name;
        double PositionMatrix[3];
        double PositionX;
        double PositionY;
        double PositionZ;
        cout << "Please enter position x:";
        cin >> PositionX;
        PositionMatrix[0] = PositionX;
        cout << "Please enter position y:";
        cin >> PositionY;
        PositionMatrix[1] = PositionY;
        cout << "Please enter position z:";
        cin >> PositionZ;
        PositionMatrix[2] = PositionZ;
        
        // cout << "Your chose to specify theis: " << x << "\n"; // Display the input value 
        if (x == 'Y' and y == 'V')
        {
            //Exercise 1 

            double OrientationMatrix[3];
            double OrientationX;
            double OrientationY;
            double OrientationZ;
            cout << "Please enter orientation x:";
            cin >> OrientationX;
            OrientationMatrix[0] = OrientationX;
            cout << "Please enter orientation y:";
            cin >> PositionY;
            OrientationMatrix[1] = OrientationY;
            cout << "Please enter orientation z:";
            cin >> PositionZ;
            OrientationMatrix[2] = OrientationZ;
        
            double MaximumVelocity ;
            cout << "Please enter the fraction of the maximum velocity:";
            cin >> MaximumVelocity;
            double ExecutionTime = 0.0;


            my_NAO_ControlInstance.callMoveJointsService(joint_name, PositionMatrix, OrientationMatrix, MaximumVelocity, ExecutionTime);
            ros::Duration(1.0).sleep();  // Sleep for 1 second (adjust as needed)
            ros::spinOnce();
        }
        else if (x == 'Y' and y == 'T')
        {

            double OrientationMatrix[3];
            double OrientationX;
            double OrientationY;
            double OrientationZ;
            cout << "Please enter orientation x:";
            cin >> OrientationX;
            OrientationMatrix[0] = OrientationX;
            cout << "Please enter orientation y:";
            cin >> PositionY;
            OrientationMatrix[1] = OrientationY;
            cout << "Please enter orientation z:";
            cin >> PositionZ;
            OrientationMatrix[2] = OrientationZ;
            double MaximumVelocity = 0.0;
            double ExecutionTime ;
            cout << "Please enter the execution time:";
            cin >> ExecutionTime;


            my_NAO_ControlInstance.callMoveJointsService(joint_name, PositionMatrix, OrientationMatrix, MaximumVelocity, ExecutionTime);
            ros::Duration(1.0).sleep();  // Sleep for 1 second (adjust as needed)
            ros::spinOnce();

            
        }
        else if (x == 'N' and y == 'V')
        {

            double OrientationMatrix[3] = {0.0, 0.0, 0.0};
            double MaximumVelocity ;
            cout << "Please enter the fraction of the maximum velocity:";
            cin >> MaximumVelocity;
            double ExecutionTime = 0.0;


            my_NAO_ControlInstance.callMoveJointsService(joint_name, PositionMatrix, OrientationMatrix, MaximumVelocity, ExecutionTime);
            ros::Duration(1.0).sleep();  // Sleep for 1 second (adjust as needed)
            ros::spinOnce();

        }
        else if (x == 'N' and y == 'T')
        {
            double OrientationMatrix[3] = {0.0, 0.0, 0.0};
            double MaximumVelocity = 0.0;
            double ExecutionTime ;
            cout << "Please enter the execution time:";
            cin >> ExecutionTime;


            my_NAO_ControlInstance.callMoveJointsService(joint_name, PositionMatrix, OrientationMatrix, MaximumVelocity, ExecutionTime);
            ros::Duration(1.0).sleep();  // Sleep for 1 second (adjust as needed)
            ros::spinOnce();

        }
        else
        {
            cout << "Please input a correct letter. \n";
        }

        
    }
    return 0;

}