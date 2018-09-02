#include "ros/ros.h"
/*
beginner_tutorials/AddTwoInts.h is the header file generated from the srv file that we created earlier.

srv: an srv file describes a service. It is composed of two parts: a request and a response.
request

Example: 
int64 A
int64 B
---
int64 Sum


*/

#include "beginner_tutorials/AddTwoInts.h"

/**

Step 1. 
The callback’s job is to fill in the data members of the Response object.

package_name::service_type::Request
package_name::service_type::Response
*/
bool add(beginner_tutorials::AddTwoInts::Request &req,
         beginner_tutorials::AddTwoInts::Response &res){
  res.sum = req.a + req.b;
  ROS_INFO("request : x =%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int) res.sum);
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;
  /*
  To associate the callback function with a service name, and to offer the service to other nodes, we must advertise the service
  ros::ServiceServer nh.advertiseService(const std::string& service, <callback>);
  */
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");

  ros::spin();
  return 0; 

}

