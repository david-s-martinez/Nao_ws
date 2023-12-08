#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_msgs/ColorRGBA.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>

bool stop_thread = false;

void spinThread()
{
	while(!stop_thread)
	{
		ros::spinOnce();
		// ROS_INFO_STREAM("Spinning the thing!!");
	}
}


class Nao_control
{
public:
	// ros handler
	ros::NodeHandle nh_;

	// subscriber to bumpers states
	ros::Subscriber bumper_sub;

	// subscriber to head tactile states
	ros::Subscriber tactile_sub;

	//publisher for nao speech
	ros::Publisher speech_pub;

	//publisher for nao leds
	ros::Publisher leds_pub;

	//publisher for nao vocabulary parameters
	ros::Publisher voc_params_pub;

	//client for starting speech recognition
	ros::ServiceClient recog_start_srv;

	//client for stoping speech recognition
	ros::ServiceClient recog_stop_srv;

	// subscriber to speech recognition
	ros::Subscriber recog_sub;

	// publisher to nao walking
	ros::Publisher walk_pub;

	//subscriber for foot contact
	ros::Subscriber footContact_sub;

	boost::thread *spin_thread;

	Nao_control()
	{

		// subscribe to topic bumper and specify that all data will be processed by function bumperCallback
		bumper_sub=nh_.subscribe("/bumper",1, &Nao_control::bumperCallback, this);

		// subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
		// tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::tactileCallback, this);

		speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

		leds_pub= nh_.advertise<naoqi_bridge_msgs::BlinkActionGoal>("/blink/goal", 1);

		voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);

		recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");

		recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");

		recog_sub=nh_.subscribe("/word_recognized",1, &Nao_control::speechRecognitionCB, this);

		footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &Nao_control::footContactCB, this);

		walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);

		stop_thread=false;
		spin_thread=new boost::thread(&spinThread);
	}
	~Nao_control()
	{
		stop_thread=true;
		sleep(1);
		spin_thread->join();
	}

	void footContactCB(const std_msgs::BoolConstPtr& contact)
	{
		/*
		 * TODO tutorial 3
		 */
	}

	void speechRecognitionCB(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
	{
		/*
		 * TODO tutorial 3
		 */
	}

	//Exerciise 1 
	void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& bumperState)
	{

		// Define the message for eye blinking
		naoqi_bridge_msgs::BlinkActionGoal blink_message;
		// Define the colors for blinking
		std_msgs::ColorRGBA color;

		// 1 is Left Bumper and 0 is Right Bumper 
		// state is 1 == Bumper is Pressed 
		// state is 0 == Bumper is Released 
		if(bumperState->bumper == 1 && bumperState->state == 1)
		{
			ROS_INFO_STREAM("You Pressed Left, Blinking Red");
			ros::Duration duration(2);
			color.r = 1.0;
			color.g = 0.0; 
			color.b = 0.0; 
			color.a = 1.0; 
			blink_message.goal.colors.push_back(color);
			blink_message.goal.blink_duration = duration;
			blink_message.goal.blink_rate_mean = 2;
			blink_message.goal.blink_rate_sd = 0.1; 
			leds_pub.publish(blink_message);

		}
		else if(bumperState->bumper == 1 && bumperState->state == 0)
		{
			ROS_INFO_STREAM("You Released Left, Stop Blinking Red");
			// Define the message for eye blinking
        	naoqi_bridge_msgs::BlinkActionGoal blink_message;
			// Define the colors for blinking
			//std_msgs::ColorRGBA color
			ros::Duration duration(4);
			color.r = 0.0;
			color.g = 0.0; 
			color.b = 0.0; 
			color.a = 0.0; 
			blink_message.goal.colors.push_back(color);
			blink_message.goal.blink_duration = duration;
			blink_message.goal.blink_rate_mean = 5;
			blink_message.goal.blink_rate_sd = 0.1; 
			leds_pub.publish(blink_message);

		}

		if(bumperState->bumper == 0 && bumperState->state == 1)
		{
			ROS_INFO_STREAM("You Pressed Right, Blinking Green");
			// Define the message for eye blinking
        	naoqi_bridge_msgs::BlinkActionGoal blink_message;
			// Define the colors for blinking
			//std_msgs::ColorRGBA color
			ros::Duration duration(4);
			color.r = 0.0;
			color.g = 1.0; 
			color.b = 0.0; 
			color.a = 1.0; 
			blink_message.goal.colors.push_back(color);
			blink_message.goal.blink_duration = duration;
			blink_message.goal.blink_rate_mean = 5;
			blink_message.goal.blink_rate_sd = 0.1; 
			leds_pub.publish(blink_message);

		}
		else if(bumperState->bumper == 0 && bumperState->state == 0)
		{
			ROS_INFO_STREAM("You Released Right, Stop Blinking Green");
			// Define the message for eye blinking
        	naoqi_bridge_msgs::BlinkActionGoal blink_message;
			// Define the colors for blinking
			//std_msgs::ColorRGBA color
			ros::Duration duration(4);
			color.r = 0.0;
			color.g = 0.0; 
			color.b = 0.0; 
			color.a = 0.0; 
			blink_message.goal.colors.push_back(color);
			blink_message.goal.blink_duration = duration;
			blink_message.goal.blink_rate_mean = 5;
			blink_message.goal.blink_rate_sd = 0.1; 
			leds_pub.publish(blink_message);

		}


	}

	// void tactileCallback(const naoqi_bridge_msgs::TactileTouch::ConstPtr& tactileState)
	// {
	// 	/*
	// 	 * TODO tutorial 3
	// 	 */
	// }


	void main_loop()
	{
		ros::Rate rate_sleep(10);
		while(nh_.ok())
		{
			/*
			 * TODO tutorial 3
			 */
			rate_sleep.sleep();
		}
	}

	void walker(double x, double y, double theta)
	{
		/*
		 * TODO tutorial 3
		 */
	}

	void stopWalk()
	{
		/*
		 * TODO tutorial 3
		 */
	}

};
int main(int argc, char** argv)
{
	ros::init(argc, argv, "tutorial_control");

	ros::NodeHandle n;
	ros::Rate rate_sleep(20);
	Nao_control ic;
	ic.main_loop();
	return 0;

}
