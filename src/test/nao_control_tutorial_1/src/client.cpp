#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "nao_control_tutorial_1/MoveJoints.h"
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

private:
    // Service client for move_joints_tutorial service
    ros::ServiceClient move_joints_client;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nao_tutorial_control_1");
    Nao_control ic;
    for (int i = 0; i < 3; ++i)
    {
        // Call the service with different angles (-90.0, -45.0, 0.0)
        std::string joint_name_1 = "RShoulderRoll";
        std::string joint_name_2 = "RShoulderPitch";
        double angle1 = -90.0 + i *30.0;  // Adjust the angle increment as needed
        double angle2 = 0.0 + i *25.0;  // Adjust the angle increment as needed
        double speed = 0.1;

        // Call the service with the current joint, angle, and speed
        ic.callMoveJointsService(joint_name_1, angle1, speed);
        ros::Duration(1.0).sleep();  // Sleep for 2 seconds (adjust as needed)
        ic.callMoveJointsService(joint_name_2, angle2, speed);
        // Add a delay or other processing if needed
        ros::Duration(1.0).sleep();  // Sleep for 2 seconds (adjust as needed)
    }

    ros::spin();
    return 0;
}
