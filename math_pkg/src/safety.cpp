/*! \file */

#include <algorithm>
#include <iostream>
#include <thread>
#include "math_pkg/Safety.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "utilities.h"
/*#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>*/
#include <ctime>

/*! Availability flag for joint angles vector.*/
bool readyq;
/*! Availability flag for joint velocities vector.*/
bool readyqdot;
/*! Array whose i-th element keeps the current position correction pole for joint i if joint i is being brought back from being close to a joint limit, 0 otherwise.*/
double currentAnglePole[NJOINTS];
/*! Array whose i-th element keeps the current position correction pole for joint i if joint i is being slowed down because it was close to its saturation velocity, 0 otherwise.*/
double currentVelPole[NJOINTS];
/*! Array of joint angle values.*/
double q[NJOINTS];
/*! Array of joint velocity values..*/
double qdot[NJOINTS];

/*void safetyCallback(const JointStateConstPtr& msgq,
const JointStateConstPtr& msgqdot) {
	vector<double> rcvd_q = msgq->position;
	vector<double> rcvd_qdot = msgqdot->velocity;
	std::copy(rcvd_q.begin(), rcvd_q.end(), q);
	std::copy(rcvd_qdot.begin(), rcvd_qdot.end(), q);
	readyq = readyqdot = true;
}*/



/*! Callback function for joint angles.
    \param msg The received joint angles vector.
*/
void safetyCallbackq(const sensor_msgs::JointState &msg) { 
	vector<double> rcvd_q = msg.position;
	std::copy(rcvd_q.begin(), rcvd_q.end(), q);
	readyq = true;
}



/*! Callback function for joint velocities.
    \param msg The received joint velocities vector.
*/
void safetyCallbackqdot(const sensor_msgs::JointState &msg) {
	vector<double> rcvd_qdot = msg.velocity;
	std::copy(rcvd_qdot.begin(), rcvd_qdot.end(), q);
	readyqdot = true;
}



/*! Function that computes the derivative of the task vector and the diagonal of the activation matrix
    \param rdot Vector passed by reference; on return it will contain the derivative of the task vector.
	\param Adiag Vector passed by reference; on return it will contain the elements of the diagonal of the activation matrix A.
*/
void safetyLoop(VectorXd& rdot, VectorXd& Adiag) {
	bool angleOk;
	double rdot_i,Adiag_i;
	for(short i = 0; i<NJOINTS; i++) { // for all joints
		// Compute and store the i-th element of rdot and of the diagonal of A, based on the angle constraint task.
	    angleOk = jointConstr(q[i],QMIN[i],QMAX[i],JOINTS_MARGIN,JOINTS_MAXPOLE,JOINTS_MAJ,currentAnglePole[i],true,rdot_i,Adiag_i);

		// If joint i has a safe rotation angle, check for the safety of its velocity. Otherwise, the angle correction task will automatically bring the joint velocity to 0.
		if (angleOk) {

			// Compute and store the i-th element of rdot and of the diagonal of A, based on the velocity constraint task.
			jointConstr(qdot[i],-QDOTMAX[i],QDOTMAX[i],VEL_MARGIN,VEL_MAXPOLE,VEL_MAJ,currentVelPole[i],false,rdot_i,Adiag_i);
		}
		// Update i-th element of the output vectors.
	    rdot(i) = rdot_i;
	    Adiag(i) = Adiag_i;
	}
}



/*! Service function for the Safety service, which computes the partial joint velocities based on the safety task.
    \param req Empty.
	\param res Safety-constrained joint velocities.
    \return true if client-service call succeeded, false otherwise.
*/
bool computePartialqdot(math_pkg::Safety::Request  &req, math_pkg::Safety::Response &res) {
	if (!(readyq && readyqdot)) { // at least one subscribtion data is missing
		readyq = readyqdot = false; // reset availability flag
    	ROS_ERROR("safety service could not run: missing data.");
		return false;
	}

	readyq = readyqdot = false; // reset availability flag
	VectorXd partial_qdot(NJOINTS),rdot(NJOINTS),Adiag(NJOINTS);

	// Obtain the derivative of the task vector and the activation values, stored in 1D vectors.
	safetyLoop(rdot,Adiag);
    MatrixXd A = Adiag.asDiagonal(); // store the diagonal of A into a sparse matrix structure (needed for the following computations)

	// For the safety task, the Jacobian is the identity and Q is the zero matrix.
	MatrixXd pinvJ = regPinv(ID_MATRIX_NJ,A,ZERO_MATRIX_NJ,ETA);

	MatrixXd Q1 = ID_MATRIX_NJ - pinvJ; // Q2 will be needed for the tracking task.
	partial_qdot = pinvJ * rdot;

	// Store Q2 into a 1D vector so that it can be sent to the client in a Float64MultiArray object.
	Map<MatrixXd> Q1v (Q1.data(), NJOINTS*NJOINTS,1);

	// Fill response object.
	res.qdot.velocity = vector<double> (partial_qdot.data(), partial_qdot.data() + partial_qdot.size());
	res.Q1.data = vector<double> (Q1v.data(), Q1v.data() + Q1v.size());

	return true; // request successful
}



/*! Main function of the node.
*/  
int main(int argc,char **argv) {

    ros::init(argc, argv, "safety_server"); // initialize node
    ros::NodeHandle n; // define node handle
    int queSize = 10;
	
	// With sync:
	/*Subscriber<JointState> subq(n,"cmdtopic",queSize);
	Subscriber<JointState> subqdot(n,"joint_vel",queSize);
	typedef sync_policies::ApproximateTime<JointState, JointState> MySyncPolicy;
  	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subq, subqdot);
  	sync.registerCallback(boost::bind(&safetyCallback, _1, _2));*/
	
	// Without sync:
	ros::Subscriber sub1 = n.subscribe("logtopic", queSize, safetyCallbackq); // subscribe to VREP
    ros::Subscriber sub2 = n.subscribe("cmdtopic", queSize, safetyCallbackqdot); // subscribe to weighter
    
	ros::ServiceServer service = n.advertiseService("safety", computePartialqdot); // activate safety service

    cout << "SAFETY WILL NOW PROCEED TO SPIN" << endl;
    ros::spin();
    return 0;
}