/**
 * @file logger
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
 * This is the subscriber node that writes on alog file the message received on logtopic topic by the simulator.
 */
/*! \file */
#include "stdlib.h"
#include "unistd.h"
#include "stdio.h"
#include <string>
#include <fstream>
#include <iostream>
#include "fcntl.h"
#include "math.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"		//type of msgs to be included
#include "sensor_msgs/JointState.h" //type of msgs to be included

using namespace std;
//! The log file identifier is here declared
FILE *myfile;

//! The Callback function allows to write both a timestamp and the joint configuration values in the log file

void logtopicCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
	char st[50];
	memset(st,0,sizeof(st));
	double stamp = msg->header.stamp.toSec();
	sprintf(st,"%f",stamp);

	fprintf(myfile, "%s %lf %lf %lf %lf %lf %lf %lf \n", st, msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5], msg->position[6]);

}

//! The main function opens the log file, initiates the ROS subscriber to logtopic topic and calls the Callback function whenever some new data are avaialble.
int main(int argc, char **argv)
{

	myfile = fopen("logger.txt", "w");

	ros::init(argc, argv, "logger"); //initialize the ros sys, giving it a name

	ros::NodeHandle n; //obj of NodeHandle class

	ros::Subscriber sub = n.subscribe("logtopic", 1000, logtopicCallback); //subscriber declaration

	ros::spin();

	fclose(myfile);

	return 0;
}
