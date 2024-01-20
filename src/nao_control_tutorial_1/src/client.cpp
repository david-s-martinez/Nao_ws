#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "nao_control_tutorial_1/MoveJoints.h"
#include "nao_control_tutorial_1/InterpolationJoints.h"
#include <string.h>

using namespace std;

class Nao_control
{
public:
    // ros handler
    ros::NodeHandle nh_;

    // subscriber to joint states
    ros::Subscriber sensor_data_sub;

    Nao_control()
    {
        sensor_data_sub = nh_.subscribe("/joint_states", 1, &Nao_control::sensorCallback, this);

        // TODO: Initialize the service client
        move_joints_client = nh_.serviceClient<nao_control_tutorial_1::MoveJoints>("move_joints_tutorial");
        move_joints_interpolation_client = nh_.serviceClient<nao_control_tutorial_1::InterpolationJoints>("move_joints_tutorial_interpolation");
    }

    ~Nao_control()
    {
    }

    // handler for joint states
    void sensorCallback(const sensor_msgs::JointState::ConstPtr &jointState)
    {
        // TODO: Process joint state data
    }

    // TODO: Create functions for each task

    // Function to call move_joints_tutorial service
    bool callMoveJointsService(const std::string &name, double angle, double speed)
    {
        // Create the service message
        nao_control_tutorial_1::MoveJoints srv;
        srv.request.name = name;
        srv.request.angle = angle;
        srv.request.speed = speed;

        // Call the service
        move_joints_client.call(srv);
    }

        // Function to call move_joints_tutorial_interpolation service
    bool callMoveJointsInterpolationService(const std::string joint_names[1], double anglesList[5], double time_assigned[5], bool isAbsolute)
    {
        // Create the service message
        nao_control_tutorial_1::InterpolationJoints srv;
        for (size_t i = 0; i < 1; ++i)
        {
            srv.request.joint_names[i] = joint_names[i];
        }

        for (size_t i = 0; i < 5; ++i)
        {
            srv.request.anglesList[i] = anglesList[i];
            srv.request.time_assigned[i] = time_assigned[i];
        }

        srv.request.isAbsolute = isAbsolute;
        // Call the service
        move_joints_interpolation_client.call(srv);
    }

private:
    // Service client for move_joints_tutorial service
    ros::ServiceClient move_joints_interpolation_client;
    ros::ServiceClient move_joints_client;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nao_tutorial_control_1");
    Nao_control ic;
    while (true)
    {
        char x; 
        cout << "Choose the desire mode (AngleInterpolation or simple AnglesSetting). \n";
        cout << "Press I for AngleInterpolation and A for AngleSetting: \n";
        cin >> x; // Get user input from the keyboard
        cout << "Your mode is: " << x << "\n"; // Display the input value 
        if (x == 'I')
        {
            //Exercise 2
            std::string joint_names[1] = {"LShoulderRoll"};
            double anglesList[5] = {6.0, 12.0, 18.0, 24.0, 30.0};
            double time_assigned[5] = {0.4, 0.8, 1.2, 1.6, 2.0};
            bool isAbsolute = true;

            ic.callMoveJointsInterpolationService(joint_names, anglesList, time_assigned, isAbsolute);
            ros::Duration(1.0).sleep();  // Sleep for 1 second (adjust as needed)
            ros::spinOnce();
        }
        else if (x == 'A')
        {
            // Exercise 1 
            for (int i = 0; i < 3; ++i)
            {
                // Call the service with different angles (-90.0, -45.0, 0.0)
                std::string joint_name_1 = "RShoulderRoll";
                std::string joint_name_2 = "RShoulderPitch";
                double angle1 = -90.0 + i *30.0;  // Adjust the angle increment as needed
                double angle2 = 0.0 + i *25.0;  // Adjust the angle increment as needed
                double speed = 0.5;

                // Call the service with the current joint, angle, and speed
                ic.callMoveJointsService(joint_name_1, angle1, speed);
                // ros::Duration(1.0).sleep();  // Sleep for 2 seconds (adjust as needed)
                // ic.callMoveJointsService(joint_name_2, angle2, speed);
                // Add a delay or other processing if needed
                // ros::Duration(1.0).sleep();  // Sleep for 2 seconds (adjust as needed)
                ros::spinOnce();
            }
        }
        else
        {
            cout << "Please input a correct method \n";
        }

        
    }

    return 0;
}