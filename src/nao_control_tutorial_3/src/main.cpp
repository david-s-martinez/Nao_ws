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
#include <naoqi_bridge_msgs/HandTouch.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
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
	//client for stoping walking
	ros::ServiceClient stop_walk_srv;

	// subscriber to speech recognition
	ros::Subscriber recog_sub;

	// publisher to nao walking
	ros::Publisher walk_pub;

	//subscriber for foot contact
	ros::Subscriber footContact_sub;

	boost::thread *spin_thread;

	std::vector<std::string> recognized_words;

	Nao_control()
	{

		// subscribe to topic bumper and specify that all data will be processed by function bumperCallback
		bumper_sub=nh_.subscribe("/bumper",1, &Nao_control::bumperCallback, this);

		// subscribe to topic tactile_touch and specify that all data will be processed by function head tactileCallback
		tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::headTactileCallback, this);

		// subscribe to topic tactile_touch and specify that all data will be processed by function head tactileCallback
		//tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::handTactileCallback, this);


		speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

		leds_pub= nh_.advertise<naoqi_bridge_msgs::BlinkActionGoal>("/blink/goal", 1);

		voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);

		recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");

		recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");
		stop_walk_srv=nh_.serviceClient<std_srvs::Empty>("/stop_walk_srv");

		recog_sub=nh_.subscribe("/word_recognized",10, &Nao_control::speechRecognitionCB, this);

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
	void setVocabulary() 
	{
		naoqi_bridge_msgs::SetSpeechVocabularyActionGoal vocab_msg;

		// Add words to your vocabulary
		vocab_msg.goal.words.push_back("hello");
		vocab_msg.goal.words.push_back("goodbye");
		vocab_msg.goal.words.push_back("yes");
		vocab_msg.goal.words.push_back("no");
		// ... add other words as needed ...

		// Publish the vocabulary
		voc_params_pub.publish(vocab_msg);

		ROS_INFO("Published new speech vocabulary.");
	}
	void speechRecognitionCB(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
	{
		for (size_t i = 0; i < msg->words.size(); ++i) 
		{
			std::string recognized_word = msg->words[i];
			float confidence = msg->confidence_values[i];

			//Store the words 
			recognized_words.push_back(recognized_word);
			std::cout << "Recognized: " << recognized_word << " with confidence: " << confidence << std::endl;
    	}
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
			ros::Duration duration(3);
			color.r = 0.0;
			color.g = 1.0; 
			color.b = 0.0; 
			color.a = 1.0; 
			blink_message.goal.colors.push_back(color);
			blink_message.goal.blink_duration = duration;
			blink_message.goal.blink_rate_mean = 2;
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
			ros::Duration duration(3);
			color.r = 0.0;
			color.g = 0.0; 
			color.b = 0.0; 
			color.a = 0.0; 
			blink_message.goal.colors.push_back(color);
			blink_message.goal.blink_duration = duration;
			blink_message.goal.blink_rate_mean = 2;
			blink_message.goal.blink_rate_sd = 0.1; 
			leds_pub.publish(blink_message);

		}


	}

	void headTactileCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& headtactileState)
	{

		// button = 1 Tactile Front - state = 1 -> Pressed
		if(headtactileState->button == 1 && headtactileState->state == 1)
		{

			// Set speech vocabulary first
			ROS_INFO_STREAM("Vocab Storing");
        	setVocabulary();
			// Start speech recognition
			ROS_INFO_STREAM("Speech Recognition Starts");
			std_srvs::Empty srv;
			recog_start_srv.call(srv);

		}
		// button = 2 Tactile Middle - state = 1 -> Pressed
		else if(headtactileState->button == 2 && headtactileState->state == 1)
		{
			//Stop speech recognition
			ROS_INFO_STREAM("Speech Recognition Stops");
			std_srvs::Empty stop_srv;
			recog_stop_srv.call(stop_srv);

			if (recognized_words.empty()){
				naoqi_bridge_msgs::SpeechWithFeedbackActionGoal words;
				words.goal.say = "stop playing with my buttons";
				speech_pub.publish(words);
			}
			else{
				for (int i = 0; i < recognized_words.size(); ++i){
					naoqi_bridge_msgs::SpeechWithFeedbackActionGoal words;
					ROS_INFO_STREAM("Talking"); 
					words.goal.say = recognized_words[i];
					speech_pub.publish(words);
					ros::Duration(1.0).sleep();

				}
				recognized_words.clear();
			}


		}
		else if(headtactileState->button == 3 && headtactileState->state == 1){
			stopWalk();
			walker(0.5,0.0,0.0);
			walker(0.0,0.0,1.58);
			walker(0.0,0.0,-1.58);
			walker(0.0,0.0,-1.58);
			walker(0.0,0.0,1.58);
			stopWalk();

		}
	}

	void handTactileCallback(const naoqi_bridge_msgs::HandTouch::ConstPtr& tactileState)
	{

	}

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
		geometry_msgs::Pose2D pose;
		pose.x = x;
		pose.y = y;
		pose.theta = theta;
		walk_pub.publish(pose);
		ros::Duration(5.0).sleep();
	}

	void stopWalk()
	{
		std_srvs::Empty stop_srv;
		stop_walk_srv.call(stop_srv);
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
