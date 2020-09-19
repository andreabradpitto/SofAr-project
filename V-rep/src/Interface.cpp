/**
 * \file Interface
 * \author  Elena Merlo - Matteo Palmas
 * \version 1.0
 *
 * \section LICENSE
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
 * \section DESCRIPTION
 *
 * This is the script encoding the user interface to make easier the simulation handling.
 */
 /*! \file */

#include "stdlib.h"
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/JointState.h" //type of msgs to be included
#include "string.h"
#include "sys/types.h"
#include "signal.h"

//! The main function acts as a user interface for simulation handling thanks to ROS publishers. 

int main(int argc, char **argv) 
  {

  ros::init(argc, argv, "Test"); //initialize the ros sys, giving it a name 

  ros::NodeHandle n1; //obj of NodeHandle class

  ros::Publisher SimPub = n1.advertise<std_msgs::Int8>("handleSimulation", 1000); //publisher advertisement 
  ros::Publisher SimStatePub = n1.advertise<sensor_msgs::JointState>("default_state", 1000); //publisher advertisement 
  ros::Publisher ExitPub = n1.advertise<std_msgs::Int8>("exitSimulation", 1000); //publisher advertisement 
  
  char string[20];
  
  ros::Rate loop_rate(10); //loop rate set at 10 Hz
  printf("To activate an action, digit your command and press enter.\nFor a list of commands use 'help' and press enter\n");
  
  //while loop -> we want to continue to publish messages, as long the node is alive
  while (ros::ok()) 	
  {
  		
    	
    	//Main body
  		printf("Digit your command and press enter: ");
  		scanf("%s",string);

		//! The command 'help' is for knowing all the possible interface functions
  		if(strcmp(string,"help")==0){
  			printf("- start\n- calibration\n- pause\n- stop\n- set_default\n- exit\n");

		//! The command 'start' is for starting the simulation
  		}else if(strcmp(string,"start")==0){
	  		std_msgs::Int8 msg;
	 		msg.data=1;
	  		//command message publishing
			SimPub.publish(msg);

		//! The command 'calibration' is for making the IMU reference system coincide with the human end effector reference system
  		}else if(strcmp(string,"calibration")==0){
	  		std_msgs::Int8 msg;
	 		msg.data=3;
	  		//command message publishing
			SimPub.publish(msg);

		//! The command 'pause' is for pausing the simulation in order to restart from the last reached configuration
  		}else if(strcmp(string,"pause")==0){
  			std_msgs::Int8 msg;
	 		msg.data=2;
	  		//command message publishing
			SimPub.publish(msg);

		//! The command 'stop' is for stopping the simulation leading the robot arm to the default configuration
  		}else if(strcmp(string,"stop")==0){
  			std_msgs::Int8 msg;
	 		msg.data=0;
	  		//command message publishing
			SimPub.publish(msg);

		//! The command 'set_default' is for setting a desired default configuration for each one of the 7 joints. In the original default configuration each joint is set to 0 
  		}else if(strcmp(string,"set_default")==0){
  			sensor_msgs::JointState msg;
  			double array[7];
  			printf("Print the joint position you want to set as default:\n");
  			msg.position.resize(7);
  			for(int i=0;i<7;i++){
  				printf("joint %d:",i+1);
  				scanf("%lf",&msg.position[i]);
  			}
	  		//command message publishing
			SimStatePub.publish(msg);

		//! The command 'exit' is for closing the user interface, all the running topics and the simulator environment
  		}else if(strcmp(string,"exit")==0){
			std_msgs::Int8 msg;
	 		msg.data=1;
	  		//command message publishing
			ExitPub.publish(msg);
				
			int parent=getppid();
			system("rosnode kill -a");
			kill(parent,SIGKILL);
  			break;
  		}else{
  			printf("Error: mistake in command definition, digit 'help' for a command list\n");
  		}

    	ros::spinOnce();

	//wait for the next cycle
    	loop_rate.sleep();
    	
  }

  return 0;
	
}
