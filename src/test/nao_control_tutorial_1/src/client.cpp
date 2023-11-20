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
        sensor_data_sub=nh_.subscribe("/joint_states",1, &Nao_control::sensorCallback, this);
    }
    ~Nao_control()
    {
    }

  //handler for joint states
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& jointState)
    {
      
    }

// TODO: create function for each task

};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "nao_tutorial_control_1");

    Nao_control ic;
    ros::spin();
    return 0;

}
