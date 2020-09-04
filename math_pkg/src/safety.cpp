/*! \file */

#include <algorithm>
#include <iostream>
#include <fstream>
#include "math_pkg/Safety.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "utilities.h"
#include <ctime>

/*! Availability flag for joint angles vector: true iff a valid q vector is available.*/
bool readyq;
/*! Availability flag for joint velocities vector: true iff a valid q vector is available.*/
bool readyqdot;
/*! Array whose i-th element keeps the current position correction pole for joint i if joint i is being brought back from being close to a joint limit, 0 otherwise.*/
double currentAnglePole[NJOINTS];
/*! Array whose i-th element keeps the current position correction pole for joint i if joint i is being slowed down because it was close to its saturation velocity, 0 otherwise.*/
double currentVelPole[NJOINTS];
/*! Array of joint angle values.*/
double q[NJOINTS];
/*! Array of joint velocity values.*/
double qdot[NJOINTS];
/*! Abs value of the position correction pole for safety task.*/
#define CORRPOLE_POS 70
/*! Abs value of the velocity correction pole for safety task.*/
#define CORRPOLE_VEL 8
/*! Soft margin of joint angles.*/
#define JOINTS_MARGIN 0.1
/*! Soft margin of joint  velocities.*/
#define VEL_MARGIN 0.01
/*! At each step, the sequence number of the received q message, for synchronization.*/
int seqtry;
/* At each step, the sequence number of the received qdot message, for synchronization.*/
int seqqdot;
/* Log file used in debug. A sequential number is printed in it every time a Safety call fails.*/
ofstream safeFailFile("safeFail.txt");
/* Counter for the Safety service failures, used in debug.*/
int safeFail = 0;

ofstream safetyqfile("safetyq.txt");


/*! Callback function for joint angles.
    \param msg The received joint angles vector.
*/
void safetyCallbackq(const sensor_msgs::JointState &msg) {
	seqtry = (int)msg.effort[0]; // q sequence number
	if (seqtry < seqqdot) return; // the received q is old, nothing to do with it

	// At this point, the received q is up to date: store it in global variable q.
	vector<double> rcvd_q = msg.velocity; // should look at the position field, but Coppelia has a problem with it...
	std::copy(rcvd_q.begin(), rcvd_q.end(), q);
	readyq = true; // q available, thus set flag to true
}



/*! Callback function for joint velocities.
    \param msg The received joint velocities vector.
*/
void safetyCallbackqdot(const sensor_msgs::JointState &msg) {
	// Store data in global variable qdot.
	vector<double> rcvd_qdot = msg.velocity;
	std::copy(rcvd_qdot.begin(), rcvd_qdot.end(), qdot);

	seqqdot = (int)msg.effort[0]; // update sequence number
	readyqdot = true; // qdot available, thus set flag to true
}


/*! Cosinoidal sigmoid function departing from zero at y+/-mrgn and gets to 1 at y, with period 2*mrgn.
    \param x Point at which the sigmoid is evaluated.
    \param y Point at which the sigmoid gets to 1.
    \param mrgn Half the sigmoid's period.
    \return the sigmoid value at x.
*/
inline double cos_sigmoid(double x,double y,double mrgn) {
	return (.5 + .5 * cos((x - y) * M_PI/mrgn));
}



/*! Function that evaluates task element derivative and activation value for quantity x.
    \param x Scalar quantity.
    \param xmin Min value of x.
    \param xmax Max value of x.
    \param mrgn Soft margin of x.
    \param cPole Absolute value of correction pole.
    \param currentPole Current correction pole for x, if any, 0 otherwise.
    \param isJoint True if x is a joint angle, false otherwise.
    \param rdot Reference to task element derivative for quantity x.
    \param Adiag Reference to activation value for quantity x.
    \return true if no correction needed to behaviour of x, false otherwise.
*/
bool jointConstr(double x,const double xmin,const double xmax,
	const double mrgn,const double cPole,
	double &currentPole,const bool isJoint,double &rdot, double &Adiag) {
    bool ok = false;

	if (x > xmax - mrgn) { // x too high
        //cout << "too high" << x << ">" << xmax - mrgn << ", is joint = " << isJoint << endl;
		if (currentPole == 0) currentPole = cPole; // if the task has just been activated, assign pole for the task
		if (isJoint) rdot = currentPole * (-x + xmax - mrgn);
		else rdot = currentPole * (-x + xmax - mrgn) * DT + x;
		if (x > xmax) Adiag = 1; // x over the limit, task activation value = 1 (full activation)
		else Adiag = cos_sigmoid(x,xmax,mrgn); // task activation value is between 0 and 1
	}
	else if (x < xmin + mrgn) { // x too low
        //cout << "too low" << x << "<" << xmin + mrgn << ", is joint = " << isJoint << endl;
		if (currentPole == 0) currentPole = cPole; // if the task has just been activated, assign pole for the task
		if (isJoint) rdot = currentPole * (-x + xmin + mrgn);
		else rdot = currentPole * (-x + xmin + mrgn) * DT + x;
		if (x < xmin) Adiag = 1; // x under the limit, task activation value = 1 (full activation)
		else Adiag = cos_sigmoid(x,xmin,mrgn); // task activation value is between 0 and 1
	}
	else { // x has a safe value
		Adiag = 0; rdot = 0;// task inactive
		ok = true;
        currentPole = 0; // task is inactive, thus no pole is necessary
	}
    return ok;
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
	    angleOk = jointConstr(q[i],QMIN[i],QMAX[i],JOINTS_MARGIN,CORRPOLE_POS,currentAnglePole[i],true,rdot_i,Adiag_i);

		// If joint i has a safe rotation angle, check for the safety of its velocity. Otherwise, the angle correction task will automatically bring the joint velocity to 0.
		if (angleOk) {

			// Compute and store the i-th element of rdot and of the diagonal of A, based on the velocity constraint task.
			jointConstr(qdot[i],-QDOTMAX[i],QDOTMAX[i],VEL_MARGIN,CORRPOLE_VEL,currentVelPole[i],false,rdot_i,Adiag_i);
		}
		// Update i-th element of the output vectors.
	    rdot(i) = rdot_i;
	    Adiag(i) = Adiag_i;
	}
	//rdot1file << rdot << endl << endl;
}



/*! Service function for the Safety service, which computes the partial joint velocities based on the safety task.
    \param req Empty.
	\param res Safety-constrained joint velocities.
    \return true if client-service call succeeded, false otherwise.
*/
bool computePartialqdot(math_pkg::Safety::Request  &req, math_pkg::Safety::Response &res) {
	if (!(readyq && readyqdot)||(seqtry != seqqdot)) { // at least one subscribtion data is missing
		readyq = readyqdot = false; // reset availability flag
    	//ROS_ERROR("safety service could not run: missing data.");
		safeFailFile << ++safeFail << endl << endl;
		return false;
	}

	readyq = readyqdot = false; // reset availability flag
	VectorXd partial_qdot(NJOINTS),rdot(NJOINTS),Adiag(NJOINTS);

	// Obtain the derivative of the task vector and the activation values, stored in 1D vectors.
	safetyLoop(rdot,Adiag);
    MatrixXd A = Adiag.asDiagonal(); // store the diagonal of A into a sparse matrix structure (needed for the following computations)

	// For the safety task, the Jacobian is the identity and Q is the zero matrix.
	double cond;
	MatrixXd pinvJ = regPinv(ID_MATRIX_NJ,A,ID_MATRIX_NJ,ETA,cond);
	MatrixXd Q1 = ID_MATRIX_NJ - pinvJ; // Q1 will be needed for the tracking task.
	partial_qdot = pinvJ * pinvJ * rdot;
	
	safetyqfile << partial_qdot << endl << endl;

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
	
	ros::Subscriber sub1 = n.subscribe("logtopic", queSize, safetyCallbackq); // subscribe to integrator
    ros::Subscriber sub2 = n.subscribe("cmdtopic", queSize, safetyCallbackqdot); // subscribe to weighter
    
	ros::ServiceServer service = n.advertiseService("safety", computePartialqdot); // activate safety service

    ros::spin(); // keep node alive
    return 0;
}