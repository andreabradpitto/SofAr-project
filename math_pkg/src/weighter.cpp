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
#include "utilities.h"

#include <ctime>

/*! Client objects needed for service calls.*/
ros::ServiceClient clients[NUM_IK_SERVICES];

/*! Cost server object.*/ // will also need transp, analytic
math_pkg::Cost costSrv;

/* Transpose server object.*/
math_pkg::IK_Jtra traSrv;

/* Transpose server object.*/
math_pkg::IK_JTA TASrv;


/*! Function that calls all the invkin modules and retrieves the computed joint velocities.
    \param qdots Vector that will contain the computed qdots, to be filled.
	\param obtained Vector that at position i contains a boolean that is true if i-th solution was retrieved, false otherwise.
	\return number of retrieved velocities vectors.
*/
int getAllqdots(vector<double> qdots[], bool obtained[]) {
	bool costObtained; // will be true if call to Cost service will succeed.

	int num_obtained = 0; // initialization
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
	else
		ROS_ERROR("Call to Cost service failed.");
	if (obtained[0]) {
		qdots[0] = traSrv.response.q_dot.velocity;
		num_obtained++;
	}
	else ROS_ERROR("Call to Jtra service failed.");
	if (obtained[1]) {
		qdots[1] = TASrv.response.q_dot.velocity;
		num_obtained++;
	}
	else ROS_ERROR("Call to J6dofs service failed.");

	for (int i = 2; i < NUM_IK_SOLUTIONS; i++) {
		obtained[i] = costObtained;
	}

	return num_obtained;
}



/*! Service function for the Safety service, which computes the partial joint velocities based on the safety task.
    \return JointState object to be published, containing the weighted joint velocities vector.
*/
int computeWeightedqdot(JointState &finalqdotState) {
	vector<double> finalqdot(NJOINTS,0); // initialize content of object to be published
	vector<double> qdots[NUM_IK_SOLUTIONS]; // will contain all qdots computed by the invkin services
	bool obt[NUM_IK_SOLUTIONS]; // i-th element is true if i-th solution was obtained, false otherwise
	int num_obtained = getAllqdots(qdots,obt); // get all computed qdots
	
	vector<double> tempqdot(NJOINTS,0); // store here temporary qdot vectors
	double weight; // will contain vector weight at each iteration
	double residual = 0;
	bool badMissedAndCostOk = false;
	int firstReceivedIdx = -1;
	for (short i = 0; i < NUM_IK_SOLUTIONS; i++) { // for all solutions
		weight = WEIGHTS[i];
		if (obt[i]) { // if it was obtained
			if (firstReceivedIdx == -1) firstReceivedIdx = i; // store first obtained solution
			if (!badMissedAndCostOk && i>1 && residual>0) badMissedAndCostOk = true; // if at least one bad one was missed but not the COST ones
			if (badMissedAndCostOk) weight = weight + residual / 4; // distribute the lost weight over the COST solutions
			
			// Store the i-th vel vector times weight into tempqdot. I.e., tempqdot = weight * qdots[i].
			transform(qdots[i].begin(), qdots[i].end(), tempqdot.begin(), [&weight](double& c){return c*weight;});

			// Add up finalqdot and the weighted i-th vector (in tempqdot) and store the result in finalqdot. I.e., finalqdot = tempqdot + finalqdot.
			transform(tempqdot.begin(), tempqdot.end(), finalqdot.begin(), finalqdot.begin(), std::plus<double>());
		}
		else residual = residual + weight;
	}

	// If some solution was obtained, some weights were lost AND the COST solutions were not obtained.
	if (firstReceivedIdx >= 0 && residual > 0 && !badMissedAndCostOk) {
			// Store the firstReceivedIdx-th vel vector times residual into tempqdot. I.e., tempqdot = residual * qdots[firstReceivedIdx].
			transform(qdots[firstReceivedIdx].begin(), qdots[firstReceivedIdx].end(), tempqdot.begin(), [&residual](double& c){return c*residual;});

			/* Add up finalqdot and the firstReceivedIdx-th vector weighted by residual and store the result in finalqdot.
			   I.e., finalqdot = tempqdot + finalqdot. */
			transform(tempqdot.begin(), tempqdot.end(), finalqdot.begin(), finalqdot.begin(), std::plus<double>());
		
	}
	saturate(finalqdot);
	if (num_obtained > 0) finalqdotState.velocity = finalqdot; // store final velocity vector into the velocity field of the object to be published.
	finalqdotState.header.stamp = ros::Time::now();
	//printVectord(finalqdot);
	return num_obtained ;
}



/*! Main function of the node.
*/  
int main(int argc,char **argv) {
    ros::init(argc, argv, "weighter"); // initialize node
    ros::NodeHandle n; // define node handle

    int queSize = 10;
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("cmdtopic", queSize); // activate qdot publisher
    ros::Rate loopRate(1/DT); // define publishing rate

	// This node acts as a client for three services.
    clients[0] = n.serviceClient<math_pkg::Cost>("cost");
    clients[1] = n.serviceClient<math_pkg::IK_Jtra>("IK_Jtransp");
    clients[2] = n.serviceClient<math_pkg::IK_JTA>("IK_JAnalytic");

    //cout << "WEIGHTER WILL NOW PROCEED TO WEIGH" << endl;
	vector<double> tosendFirst(NJOINTS,0);
	sensor_msgs::JointState toSend; // initialize object to be published
	toSend.velocity = tosendFirst;
	toSend.header.stamp = ros::Time::now();
	pub.publish(toSend);
    loopRate.sleep();

	int obt;

    while (ros::ok()) {
		obt = computeWeightedqdot(toSend);
		ROS_ERROR("%d obtained", obt);
		pub.publish(toSend); // publish weighted qdot

    	ros::spinOnce();
    	loopRate.sleep();
    }

    return 0;
}