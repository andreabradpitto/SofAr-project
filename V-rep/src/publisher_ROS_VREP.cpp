/**
 * @file publisher_ROS_VREP
 * @author  Elena Merlo - Matteo Palmas
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * https://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * This is the dummy publisher for simulation testing.
 */


#include "stdlib.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h" //type of msgs to be included

//! The main function initiates the ROS node that publishes some DUMMY input data (7 joint positions) on logtopic topic. These values can be constant or randomly generated depending on the commented portion of the code. 


int main(int argc, char **argv) 
  {

  ros::init(argc, argv, "publisher_ROS_VREP"); //initialize the ros sys, giving it a name 

  ros::NodeHandle n; //obj of NodeHandle class

  ros::Publisher chatter_pub = n.advertise<sensor_msgs::JointState>("logtopic", 1000); //publisher advertisement 

  ros::Rate loop_rate(110);//loop rate set at 110 Hz

  //while loop -> we want to continue to publish messages, as long the node is alive

  while (ros::ok()) 	
  {
    	sensor_msgs::JointState msg; //declare our message variable

    	msg.position.resize(7); //set to 7 the array size

	//constant command message (values between -1 and 1)

    msg.position[0] = -0.2;
	msg.position[1] = 0.9;
	msg.position[2] = 0.8;
	msg.position[3] = -0.58;
	msg.position[4] = 1;
	msg.position[5] = -0.3;
	msg.position[6] = 0.04;

	//random command message (values between -1 and 1)

	/*msg.position[0] =((rand()%101)/(float)100 - 0.5)*2;
	msg.position[1] =((rand()%101)/(float)100 - 0.5)*2;
	msg.position[2] =((rand()%101)/(float)100 - 0.5)*2;
	msg.position[3] =((rand()%101)/(float)100 - 0.5)*2;
	msg.position[4] =((rand()%101)/(float)100 - 0.5)*2;
	msg.position[5] =((rand()%101)/(float)100 - 0.5)*2;
	msg.position[6] =((rand()%101)/(float)100 - 0.5)*2;*/

	msg.header.stamp = ros::Time::now();	

	//command message publishing
    	chatter_pub.publish(msg);

    	ros::spinOnce();

	//wait for the next cycle
    	loop_rate.sleep();
  }

  return 0;

}


