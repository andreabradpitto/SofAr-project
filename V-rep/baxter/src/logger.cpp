#include "stdlib.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h" //type of msgs to be included

//Callback handler
void logtopicCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	//configuration message logging
	ROS_INFO("%lf %lf %lf %lf %lf %lf %lf" ,msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6]);
}

int main(int argc, char **argv)
	{  
  
	ros::init(argc, argv, "logger");  //initialize the ros sys, giving it a name 
  
	ros::NodeHandle n;  //obj of NodeHandle class
  
	ros::Subscriber sub = n.subscribe("logtopic", 1000, logtopicCallback); //subscriber declaration
  
  	ros::spin();

  	return 0;
  
}

