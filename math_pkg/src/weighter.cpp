/*! \file */

#include <algorithm>
#include <fstream>
#include <iostream>
#include "math_pkg/Cost.h"
#include "math_pkg/IK.h"
#include "math_pkg/IK_JTA.h"
#include "math_pkg/IK_Jtra.h"
#include "math_pkg/Safety.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int8.h"
#include "utilities.h"

#include <ctime>

/*! Weights for invkin solutions (sum must be 1).*/
double WEIGHTS[] = {0,0,0,1,0,0};

/*! Client objects needed for service calls.*/
ros::ServiceClient clients[NUM_IK_SERVICES];

/*! Cost server object.*/ // will also need transp, analytic
math_pkg::Cost costSrv;

/* Transpose server object.*/
math_pkg::IK_Jtra traSrv;

/* Transpose server object.*/
math_pkg::IK_JTA TASrv;

bool reset = false;
bool mustPause = false;
bool moveOn = true;
int bestIdx = 2;

int reqs = 0;
int satisf = 0;
ofstream wstats("wstats.txt");


/*! Callback function for Jacobian matrix.
    \param msg The received Jacobian matrix.
*/
void handleCallback(const std_msgs::Int8 &msg)
{
	if (msg.data == 0) reset = true;
	else if (msg.data ==1) {
			ROS_ERROR("received 1");
	 		moveOn = true;
		}
}


/*! Function that calls all the invkin modules and retrieves the computed joint velocities.
    \param qdots Vector that will contain the computed qdots, to be filled.
	\param obtained Vector that at position i contains a boolean that is true if i-th solution was retrieved, false otherwise.
	\return number of retrieved velocities vectors.
*/
int getAllqdots(vector<double> qdots[], double cost[], bool obtained[]) {
	bool costObtained; // will be true if call to Cost service will succeed.

	int num_obtained = 0; // initialization
	costSrv.request.seq.data = seq;
	// Each service call is performed in parallel.
	#pragma omp sections
		{
   			#pragma omp section
   			{
				costObtained = clients[0].call(costSrv); // call service Cost
   			}
   			#pragma omp section
			{
				obtained[0] = false;//clients[1].call(traSrv);
			}

   			#pragma omp section
			{ 
				obtained[1] = false;//clients[2].call(TASrv);
			}
		}
   	if (costObtained) { // store solutions obtained from Cost and update num_obtained
		qdots[2] = costSrv.response.qdot1opt1.velocity;
		qdots[3] = costSrv.response.qdot1opt2.velocity;
		qdots[4] = costSrv.response.qdot2opt1.velocity;
		qdots[5] = costSrv.response.qdot2opt2.velocity;
		cost[2] = costSrv.response.indicator11.data;
		cost[3] = costSrv.response.indicator12.data;
		cost[4] = costSrv.response.indicator21.data;
		cost[5] = costSrv.response.indicator22.data;
		num_obtained = num_obtained + 4;
	}
	else;
		//ROS_ERROR("Call to Cost service failed.");
	if (obtained[0]) {
		qdots[0] = traSrv.response.q_dot.velocity;
		num_obtained++;
	}
	else;// ROS_ERROR("Call to Jtra service failed.");
	if (obtained[1]) {
		qdots[1] = TASrv.response.q_dot.velocity;
		num_obtained++;
	}
	else;// ROS_ERROR("Call to J6dofs service failed.");

	for (int i = 2; i < NUM_IK_SOLUTIONS; i++) {
		obtained[i] = costObtained;
	}

	return num_obtained;
}



/*! Service function for the Safety service, which computes the partial joint velocities based on the safety task.
    \return JointState object to be published, containing the weighted joint velocities vector.
*/
int computeWeightedqdot(JointState &finalqdotState) {
	if (reqs > 0) reqs++;
 	vector<double> finalqdot(NJOINTS,0); // initialize content of object to be published
	vector<double> qdots[NUM_IK_SOLUTIONS]; // will contain all qdots computed by the invkin services
	bool obt[NUM_IK_SOLUTIONS]; // i-th element is true if i-th solution was obtained, false otherwise
	double cost[NUM_IK_SOLUTIONS];
	int num_obtained = getAllqdots(qdots,cost,obt); // get all computed qdots

	double bestCost = 0;
	bool started = false;

	// Select best qdot.
	if (!stay_still) {
		for (int i = 0; i < NUM_IK_SOLUTIONS; i++) {
			if (obt[i] && (!started || cost[bestIdx] > cost[i])) {
				started = true;
				bestIdx = i;
			}
		}
	}
	if (num_obtained > 0) {
		finalqdot = qdots[bestIdx]; // best qdot assigned
		clog << "best idx = " << bestIdx << endl;
	}
	
	saturate(finalqdot);
	if (num_obtained > 0) {
		if (reqs==0) reqs++;
		satisf++;
		if (stay_still) {
			for (int j = 0; j < NJOINTS; j++) {
				//if (abs(finalqdot[j]) < 1e-9) finalqdot[j] = 0;
			}
		}
		finalqdotState.velocity = finalqdot; // store final velocity vector into the velocity field of the object to be published.
	}
	seq = seq + 1;
	vector<double> eff(1,seq);
	finalqdotState.effort = eff;
	//printVectord(finalqdot);
	if (isnan(finalqdot[0])) num_obtained = -1;
	wstats << "seq = " << seq << ", receiv = " << reqs << ", satisf = " << satisf << endl << endl;
	return num_obtained ;
}



/*! Main function of the node.
*/  
int main(int argc,char **argv) {


    ros::init(argc, argv, "weighter"); // initialize node
    ros::NodeHandle n; // define node handle

    int queSize = 10;

    ros::Subscriber sub1 = n.subscribe("handleSimulation", queSize, handleCallback); // subscribe to Jacobian

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("cmdtopic", queSize); // activate qdot publisher
    ros::Rate loopRate(20); // define publishing rate

	// This node acts as a client for three services.
    clients[0] = n.serviceClient<math_pkg::Cost>("cost");
    clients[1] = n.serviceClient<math_pkg::IK_Jtra>("IK_Jtransp");
    clients[2] = n.serviceClient<math_pkg::IK_JTA>("IK_JAnalytic");

	/*while (ros::ok()) {
			if (moveOn) break;
			ROS_ERROR(moveOn);
	}*/
	while (ros::ok()) {
		vector<double> tosendFirst(NJOINTS,0);
		vector<double> firstEffort(1,0);
		sensor_msgs::JointState toSend; // initialize object to be published
		toSend.velocity = tosendFirst;
		toSend.header.stamp = ros::Time::now();
		toSend.effort = firstEffort;

		int obt;

    	while (ros::ok()) {
			if (moveOn) {
				if (reset) {
					reset = false; stay_still = false;
					break;
				}
				obt = computeWeightedqdot(toSend);
				/*for (int i = 0; i < NJOINTS; i++) {
					sentqdot << toSend.velocity[i] << ",";
				}*/
				if (obt == -1) break;
				if (obt == 0) ROS_ERROR("%d obtained", obt);
				pub.publish(toSend); // publish weighted qdot
				/*weightertime << ros::Time::now()<<endl<<endl;*/
			}
    		ros::spinOnce();
    		loopRate.sleep();
				//ROS_ERROR("EXIT");
			//if (mustPause) break;
    	}
		//if (obt == -1 || mustPause) break;
	}

    return 0;
}