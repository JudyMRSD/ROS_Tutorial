#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
 int main(int argc, char **argv)
 {

 	/**
 	init node "talker"

 	Creating this object registers your program as anode with the ROS master. 

 	A node is an executable that uses ROS to communicate with other nodes
 	*/
 	ros::init(argc, argv, "talker");
 	//NodeHandle is the main access point to communications with the ROS system.
 	ros::NodeHandle n;
	/**
	Publisher object  = advertise<message ty>(topic name, size of the message queue)

	advertise() returns a Publisher object serves 2 purposes: 
	1. publish messages on that topic through a call to publish().  
	2. Once all copies of the returned Publisher object are destroyed, the topic
	will be automatically unadvertised.

	Tell the master that we are going to be publishing a message of type std_msgs/String on the topic chatter. This lets the master tell any nodes listening on chatter that we are going to publish data on that topic.

	*/


	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	/**
	A ros::Rate object allows you to specify a frequency that you would like to loop at.
 	*/
 	ros::Rate loop_rate(10);

 	 /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
 	int count = 0;
 	/*
 	By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause ros::ok() to return false if that happens.
 	*/
 	while(ros::ok()){
 		/**
     * This is a message object. You stuff it with data, and then publish it.

     We broadcast a message on ROS using a message-adapted class, generally generated from a msg file. 
     */
 		//namespace :: class objectName
 		std_msgs::String msg; 

 		std::stringstream ss;

 		ss<<"hello world"<<std::cout;
 		/**
 		String message, has one member: "data". 
 		*/
 		msg.data = ss.str();

 		ROS_INFO("%s", msg.data.c_str());


    /**
     The publish(message object) function is how you send messages. 

     The type of the message object must agree with the type given as a template parameter to the advertise<>() call
     */

 		chatter_pub.publish(msg);
 		/**
		Calling ros::spinOnce() here is not necessary for this simple program, because we are not receiving any callbacks. However, if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called.
		*/

 		ros::spinOnce();
 		/**
		Now we use the ros::Rate object to sleep for the time remaining to let us hit our 10Hz publish rate. 
		*/

 		loop_rate.sleep();

 		 ++count;
  	}


  	return 0;


 }