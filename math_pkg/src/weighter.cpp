/*! \file */
#include <algorithm>
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

/*! Client objects needed for service calls.*/
ros::ServiceClient clients[NUM_IK_SERVICES];

/*! Cost server object.*/ // will also need transp, analytic
math_pkg::Cost costSrv;

/* Transpose server object.*/
math_pkg::IK_Jtra traSrv;

/* Transpose server object.*/
math_pkg::IK_JTA TASrv;

/* Boolean, true if the simulation must be reset.*/
bool reset = false;

/* Boolean, true if the simulation must be paused.*/
bool mustPause = false;

/* Boolean, true if the simulation must move on.*/
bool moveOn = false;

/* Integer variable that will contain the index of the chosen inverse kinematics solution.*/
int bestIdx = 0;


/*! Callback function for simulation signals.
    \param msg The received data.
*/
void handleCallback(const std_msgs::Int8 &msg)
{
	if (msg.data == 0) {
		reset = true;
		moveOn = false;
	}
	else if (msg.data == 1) moveOn = true;
	else if (msg.data == 2) moveOn = false;
}


/*! Function that calls all the invkin modules and retrieves the computed joint velocities.
    \param qdots Vector that will contain the computed qdots, to be filled.
	\param obtained Vector that at position i contains a boolean that is true if i-th solution was retrieved, false otherwise.
	\return number of retrieved velocities vectors.
*/
int getAllqdots(vector<double> qdots[], bool obtained[]) {

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
				obtained[0] = clients[1].call(traSrv);
			}

   			#pragma omp section
			{ 
				obtained[1] = clients[2].call(TASrv);
			}
		}

   	if (costObtained) { // store solutions obtained from Cost and update num_obtained
		qdots[2] = costSrv.response.qdot1opt1.velocity;
		qdots[3] = costSrv.response.qdot1opt2.velocity;
		qdots[4] = costSrv.response.qdot2opt1.velocity;
		qdots[5] = costSrv.response.qdot2opt2.velocity;
		num_obtained = num_obtained + 4;
	}

	if (obtained[0]) {
		qdots[0] = traSrv.response.q_dot.velocity;
		num_obtained++;
	}

	if (obtained[1]) {
		qdots[1] = TASrv.response.q_dot.velocity;
		num_obtained++;
	}

	for (int i = 2; i < NUM_IK_SOLUTIONS; i++) {
		obtained[i] = costObtained;
	}

	return num_obtained;
}



/*! Function that computes the final joint velocities.
	\param finalqdotState Joint state object to be filled in.
	\return number of retrieved velocities vectors.
*/
int computeWeightedqdot(JointState &finalqdotState) {
 	vector<double> finalqdot(NJOINTS,0); // initialize content of object to be published
	vector<double> qdots[NUM_IK_SOLUTIONS]; // will contain all qdots computed by the invkin services
	bool obt[NUM_IK_SOLUTIONS]; // i-th element is true if i-th solution was obtained, false otherwise
	int num_obtained = getAllqdots(qdots,obt); // get all computed qdots

	// Select best qdot.
	if (obt[0] && !obt[1] && !obt[2]) {
		bestIdx = 0;
	}
	else if (obt[1] && !obt[2]) {
		bestIdx = 1;
	}
	else if (obt[2]) {
		bestIdx =2;
	}

	if (num_obtained > 0) {
		finalqdot = qdots[bestIdx]; // best qdot assigned
	}
	saturate(finalqdot);
	if (num_obtained > 0) {
		finalqdotState.velocity = finalqdot; // store final velocity vector into the velocity field of the object to be published.
	}
	seq = seq + 1;
	vector<double> eff(1,seq);
	finalqdotState.effort = eff;
	if (isnan(finalqdot[0])) num_obtained = -1;
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
    ros::Rate loopRate(100); // define publishing rate

	// This node acts as a client for three services.
    clients[0] = n.serviceClient<math_pkg::Cost>("cost");
    clients[1] = n.serviceClient<math_pkg::IK_Jtra>("IK_Jtransp");
    clients[2] = n.serviceClient<math_pkg::IK_JTA>("IK_JAnalytic");

	while (ros::ok()) {
		vector<double> tosendFirst(NJOINTS,0);
		vector<double> firstEffort(1,0);
		sensor_msgs::JointState toSend; // initialize object to be published
		toSend.velocity = tosendFirst;
		toSend.effort = firstEffort;

		int obt;

    		while (ros::ok()) {
			if (moveOn) {
				obt = computeWeightedqdot(toSend);
				if (obt == -1) break;
				pub.publish(toSend); // publish weighted qdot
			}
			else if (reset) {
				reset = false; stay_still = false;
				break;
			}
    			ros::spinOnce();
    			loopRate.sleep();
    		}
	}

    return 0;
}
