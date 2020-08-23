/*! \file */

#include <algorithm>
#include <iostream>
#include "math_pkg/Cost.h"
#include "math_pkg/IK.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "utilities.h"

#include <ctime>

/*! Availability flag for Jacobian matrix.*/
bool readyq;
/*! Current Jacobian matrix.*/
VectorXd q;
/*! Client object needed to perform service calls.*/
ros::ServiceClient client;
/*! IK server object.*/
math_pkg::IK ikSrv;
/*! Sequence number for received q message.*/
int seqtry;
/*! Failure counter, used for debug.*/
int costFail = 0;
/*! Failure counter file, used for debug.*/
ofstream costFailFile("costFail.txt");


/*! Callback function for joint position.
    \param msg The received joint position vector.
*/
void costCallbackq(const sensor_msgs::JointState &msg)
{
	seqtry = (int)msg.effort[0]; // store seq number of received q
	vector<double> rcvdq = msg.velocity;
	q = Map<VectorXd>(rcvdq.data(),NJOINTS); // store q in global variable
	readyq = true;
}


/*! Function that computes the optimized velocity vectors.
    \param trackingPrecision Tracking precision of the solution obtained by IK
    \param J Current Jacobian matrix.
	\param qdot1 Joint velocity vector computed with closed loop IK of order 1.
	\param qdot2 Joint velocity vector computed with closed loop IK of order 2.
	\param Q2 Auxiliary matrix for tracking task.
    \param res Response to client of cost service.
*/
void computeCostResponse(double trackingPrecision1, double trackingPrecision2, VectorXd q,VectorXd qdot1,VectorXd qdot2,MatrixXd Q2,math_pkg::Cost::Response &res) {
    VectorXd qdot1opt1,qdot1opt2,qdot2opt1,qdot2opt2; // initialization
    //cout << "qdot1=" << qdot1 << endl;

    double cond1,cond2;

    // Each optimized velocity vector is given by: non-optimized vector + G * z, where z is a NJOINTSx1 vector.
    // Optimization n. 1: minimize qdot
    /* qdot1opt1 and qdot2opt1 are computed according to the paper "A Novel Practical Technique to Integrate Inequality Control
	 * Objectives and Task Transitions in Priority Based Control" by Casalino & Simetti, p. 20, sec. 4.4.*/
    MatrixXd IdMinusQ2 = ID_MATRIX_NJ - Q2;
    MatrixXd toPinv = Q2.transpose() * Q2 + 0.1 * IdMinusQ2.transpose()*(IdMinusQ2);
    MatrixXd temp1 = -regPinv(toPinv,ID_MATRIX_NJ,ID_MATRIX_NJ,ETA,cond1) * Q2.transpose();
    qdot1opt1 = qdot1 + Q2*temp1*qdot1; // optimize solution 1
    qdot2opt1 = qdot2 + Q2*temp1*qdot2; // optimize solution 2

    // Optimization n. 2: stay close to favourite pose (initial pose)
    MatrixXd Q2Sharp = regPinv(Q2+0.00001*ID_MATRIX_NJ,ID_MATRIX_NJ,ID_MATRIX_NJ,ETA,cond2);
    VectorXd qdot_fav = 0.1 * q;
    qdot1opt2 = qdot1 + Q2*Q2Sharp*(qdot_fav - qdot1); // optimize solution 1
    qdot2opt2 = qdot1 + Q2*Q2Sharp*(qdot_fav - qdot2); // optimize solution 2

    // Fill the response object.
    res.qdot1opt1.velocity = vector<double> (qdot1opt1.data(),qdot1opt1.data()+qdot1opt1.size());
    res.qdot1opt2.velocity = vector<double> (qdot1opt2.data(),qdot1opt2.data()+qdot1opt2.size());
    res.qdot2opt1.velocity = vector<double> (qdot2opt1.data(),qdot2opt1.data()+qdot2opt1.size());
    res.qdot2opt2.velocity = vector<double> (qdot2opt2.data(),qdot2opt2.data()+qdot2opt2.size());
    
    // Sort of normalization
    trackingPrecision1 = trackingPrecision1/0.1;
    trackingPrecision2 = trackingPrecision2/0.1;
    cond1 = cond1/1e5;
    cond2 = cond2/1e5;

    res.indicator11.data = trackingPrecision1 + cond1;
    res.indicator12.data = trackingPrecision1 + cond2;
    res.indicator21.data = trackingPrecision2 + cond1;
    res.indicator22.data = trackingPrecision2 + cond2;
}



/*! Service function for the Safety service, which computes the optimized joint velocities according to 2 optimization criteria: closeness to a preferred positional
configuration and closeness to a preferred velocity vector.
    \param req Contains current sequence number, sent by the weighter.
	\param res Four optimized joint velocities, respectively computed from IK 1 with cost function 1 and 2, and from IK 2 again with both cost functions.
    \return true if client-service call succeeded, false otherwise.
*/
bool computeOptqdot(math_pkg::Cost::Request &req, math_pkg::Cost::Response &res) {
    //clock_t begin = clock();
    if (client.call(ikSrv)) { // if Jacobian is available and the service call succeeded
        if (!(readyq && seqtry == req.seq.data)) { // Jacobian not available
            readyq = false; // reset availability flag
            costFailFile << ++costFail << endl << endl;
    	    ROS_ERROR("cost service could not run: missing data.");
    	    return false;
        }
        readyq = false; // reset availability flag

		// Map the non-optimized vectors and matrix returned by the call into Eigen library objects.
    	VectorXd qdot1 = Map<VectorXd>(ikSrv.response.qdot1.velocity.data(),NJOINTS);
    	VectorXd qdot2 = Map<VectorXd>(ikSrv.response.qdot2.velocity.data(),NJOINTS);
    	MatrixXd Q2 = Map<MatrixXd>(ikSrv.response.Q2.data.data(),NJOINTS,NJOINTS);

        // Compute optimized vectors and fill the response object.
   	 	computeCostResponse(ikSrv.response.trackingPrecision1.data,ikSrv.response.trackingPrecision2.data,q,qdot1,qdot2,Q2,res);
   	}
    else { // if Jacobian is available but the service call did not succeed
        readyq = false; // reset availability flag
    	ROS_ERROR("Call to ik service failed.");
    	return false;
    }
	/*clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "CST TOOK " << elapsed_secs << " SECS" << endl;*/
	return true;
}


/*! Main function of the node.
*/  
int main(int argc,char **argv) {
    ros::init(argc, argv, "cost_server"); // initialize node
    ros::NodeHandle n; // define node handle

    int queSize = 10;
    ros::Subscriber sub = n.subscribe("logtopic", queSize, costCallbackq); // subscribe for Jacobian

    ros::ServiceServer service = n.advertiseService("cost", computeOptqdot); // activate Cost service
    
    client = n.serviceClient<math_pkg::IK>("ik"); // Cost is a client of IK
    ros::spin();
    return 0;
}