#include "stdlib.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h" //type of msgs to be included

int main(int argc, char **argv) 
  {

  ros::init(argc, argv, "publisher_ROS_VREP"); //initialize the ros sys, giving it a name 

  ros::NodeHandle n; //obj of NodeHandle class

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("cmdtopic", 1000); //publisher advertisement 

  ros::Rate loop_rate(10); //loop rate set at 10 Hz

  //while loop -> we want to continue to publish messages, as long the node is alive
  while (ros::ok()) 	
  {
    
     	std_msgs::Float32MultiArray msg; //declare our message variable

    	msg.data.resize(7); //set to 7 the array size

	//constant command message (values between -1 and 1)

      	msg.data[0] = -0.2;
	msg.data[1] = 0.9;
	msg.data[2] = 0.8;
	msg.data[3] = -0.58;
	msg.data[4] = 1;
	msg.data[5] = -0.3;
	msg.data[6] = 0.04;

	//random command message (values between -1 and 1)

	/*msg.data[0] =((rand()%101)/(float)100 - 0.5)*2;
	msg.data[1] =((rand()%101)/(float)100 - 0.5)*2;
	msg.data[2] =((rand()%101)/(float)100 - 0.5)*2;
	msg.data[3] =((rand()%101)/(float)100 - 0.5)*2;
	msg.data[4] =((rand()%101)/(float)100 - 0.5)*2;
	msg.data[5] =((rand()%101)/(float)100 - 0.5)*2;
	msg.data[6] =((rand()%101)/(float)100 - 0.5)*2;*/

	//command message logging
  	ROS_INFO("%lf %lf %lf %lf %lf %lf %lf" ,msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6]);
	
	//command message publishing
    	chatter_pub.publish(msg);

    	ros::spinOnce();

	//wait for the next cycle
    	loop_rate.sleep();
  }

  return 0;

}


