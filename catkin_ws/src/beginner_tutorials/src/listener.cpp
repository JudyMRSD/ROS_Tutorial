#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

/**
This is the callback function that will get called when a new message has arrived on the chatter topic
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	/**
	Subscribe to the chatter topic with the master. ROS will call the chatterCallback() function whenever a new message arrives. 

	NodeHandle.subscribe(topic name, queue size, callback function)
	*/
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	/*
	 ros::spin() enters a loop, calling message callbacks as fast as possible. ros::spin() will exit once ros::ok() returns false, which means ros::shutdown() has been called, either by the default Ctrl-C handler, the master telling us to shutdown, or it being called manually. 
	*/
	ros::spin();

	return 0;

}