/*! \file */

#include <algorithm>
#include <fstream>
#include <iostream>
#include "math_pkg/IK.h"
#include "math_pkg/Safety.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "utilities.h"

/*! Linear positional gain for tracking in CLIK1.*/
#define Kpp 10
/*! Linear positional gain for tracking in CLIK2.*/
#define Kp 100
/*! Linear velocity gain for tracking.*/
#define Kv 20
/*! Rotational gain for tracking.*/
#define Krot 10
/*! First step flag.*/
bool firstStep = true;
/*! Availability flag for Jacobian matrix.*/
bool readyJ;
/*! Availability flag for error vector.*/
bool readyErr;
/*! Availability flag for tracking signals.*/
bool readyVwa;
/*! Availability flag for joint velocities vector.*/
bool readyqdot;
/*! Current Jacobian matrix.*/
MatrixXd J;
/*! Current linear part of the Jacobian matrix.*/
MatrixXd JL;
/*! Current derivative of Jacobian matrix.*/
MatrixXd JLdot;
/*! Previous value of linear part of the Jacobian matrix.*/
MatrixXd JLold;
/*! Current linear error vector.*/
VectorXd eta;
/*! Current rotational error vector.*/
VectorXd rho;
/*! Current linear velocity error vector.*/
VectorXd nu;
/*! Current linear velocity vector.*/
VectorXd v;
/*! Current angular velocity vector.*/
VectorXd w;
/*! Current linear acceleration vector.*/
VectorXd a;
/*! Current joint velocities vector.*/
VectorXd qdot;
/*! Client object needed to perform service calls.*/
ros::ServiceClient client;
/*! Safety server object.*/
math_pkg::Safety safeSrv;
/*! Threshold under which error is considered zero.*/
const double thr_still = 1e-4;


/*! Callback function for Jacobian matrix.
    \param msg The received Jacobian matrix.
*/
void ikCallbackJ(const std_msgs::Float64MultiArray &msg)
{

	vector<double> rcvdJ = msg.data;
	J = Map<MatrixXd>(rcvdJ.data(),NJOINTS,6).transpose();
	readyJ = true;
}



/*! Callback function for error vectors.
    \param msg The received error vectors.
*/
void ikCallbackErr(const std_msgs::Float64MultiArray &msg)
{
	vector<double> rcvdErr = msg.data;
	rho = Map<VectorXd>(rcvdErr.data(),3);
	eta = Map<VectorXd>(rcvdErr.data()+3,3);
	nu = Map<VectorXd>(rcvdErr.data()+6,3);
	if (abs(eta(0)) < thr_still && abs(eta(1)) < thr_still && abs(eta(2)) < thr_still &&
		abs(rho(0)) < thr_still && abs(rho(1)) < thr_still && abs(rho(2)) < thr_still) {
			eta(0) = eta(1) = eta(2) = rho(0) = rho(1) = rho(2) = nu(0) = nu(1) = nu(2) = 0;
			stay_still = true;
	}
	else stay_still = false;

	readyErr = true;
}



/*! Callback function for tracking signals.
    \param msg The received tracking signals (target linear velocity,angular velocity,linear acceleration).
*/
void ikCallbackVwa(const std_msgs::Float64MultiArray &msg)
{
	vector<double> rcvdVwa = msg.data;
	v = Map<VectorXd>(rcvdVwa.data(),3);
	w = Map<VectorXd>(rcvdVwa.data()+3,3);
	a = Map<VectorXd>(rcvdVwa.data()+6,3);
	readyVwa = true;
}



/*! Callback function for joint velocities.
    \param msg The received joint velocities vector.
*/
void ikCallbackqdot(const sensor_msgs::JointState &msg)
{
	vector<double> rcvdqdot = msg.velocity;
	qdot = Map<VectorXd>(rcvdqdot.data(),NJOINTS);
	readyqdot = true;
}



/*! Function that computes non-optimized qdot with 2 CLIK algorithms.
    \param partialqdot Safety task based qdot vector.
    \param Q1 Auxiliary matrix for tracking task.
    \param J Jacobian matrix.
    \param JL Linear Jacobian matrix.
    \param JLdot Derivative of linear Jacobian matrix.
    \param qdot Previous qdot vector.
    \param eta Linear error vector.
    \param rho Rotational error vector.
    \param etadot Linear velocity error vector.
    \param v Target velocity vector.
    \param w Target angular velocity vector.
    \param a Target acceleration vector.
    \param qdot1 Reference to CLIK 1st order solution, to be filled.
    \param qdot2 Reference to CLIK 2nd order solution, to be filled.
*/
void computeqdot(VectorXd partialqdot,MatrixXd Q1,MatrixXd J,MatrixXd JL,
    MatrixXd JLdot,VectorXd qdot,VectorXd eta,VectorXd rho,VectorXd etadot,
    VectorXd v,VectorXd w,VectorXd a,VectorXd &qdot1,VectorXd &qdot2) {
    VectorXd ve1 = v + Kpp*eta; // ee lin velocity for CLIK1
    VectorXd ve2 = DT*(a - JLdot*qdot + Kv*etadot + Kp*eta) + JL*qdot; // ee lin velocity for CLIK2
    VectorXd xedot1 = VectorXd(6);
    VectorXd xedot2 = VectorXd(6);
    xedot1 << ve1,w+Krot*rho; // ee velocity for CLIK1
    xedot2 << ve2,w+Krot*rho; // ee velocity for CLIK2
	double cond;
	/* qdots are computed according to the paper "A Novel Practical Technique to Integrate Inequality Control
	 * Objectives and Task Transitions in Priority Based Control" by Casalino & Simetti, pp. 16-17, sec. 3.4. */
    MatrixXd JTimesQ1 = J*Q1;
    MatrixXd pinvAux = regPinv(JTimesQ1,ID_MATRIX_SPACE_DOFS,Q1,ETA,cond);
    MatrixXd pinvQZero = regPinv(JTimesQ1,ID_MATRIX_SPACE_DOFS,ID_MATRIX_NJ,ETA,cond);
    MatrixXd W2 = JTimesQ1*pinvAux;
    MatrixXd tempProduct1 = Q1*pinvQZero*W2;
    MatrixXd tempProduct2 = J*partialqdot;

    qdot1 = partialqdot + tempProduct1 * (xedot1 - tempProduct2);
    qdot2 = partialqdot + tempProduct1 * (xedot2 - tempProduct2);
}


/*! Service function for the Safety service, which computes the partial joint velocities based on the safety task.
    \param req Empty.
	\param res Non-optimized joint velocities.
    \return true if client-service call succeeded, false otherwise.
*/
bool computeIKqdot(math_pkg::IK::Request  &req, math_pkg::IK::Response &res) {
    //clock_t begin = clock(); // timing
    if (client.call(safeSrv)) { // if all subscription data is available and service call succeeded
		if (!(readyJ && readyErr && readyVwa && readyqdot)) { // at least one subscribtion data is missing
			readyJ = readyErr = readyVwa = readyqdot = false; // reset availability flags
    		ROS_ERROR("ik service could not run: missing data.");
    		return false;
		}
		readyJ = readyErr = readyVwa = readyqdot = false; // reset availability flags
		double cond;

		// Map the vectors returned by the call into Eigen library objects.
    	VectorXd partialqdot = Map<VectorXd>(safeSrv.response.qdot.velocity.data(),NJOINTS);
    	MatrixXd Q1 = Map<MatrixXd>(safeSrv.response.Q1.data.data(),NJOINTS,NJOINTS);

		// Compute Q2 matrix and store it in std::vector
		MatrixXd Q2 = Q1*(ID_MATRIX_NJ - regPinv(J*Q1,ID_MATRIX_SPACE_DOFS,ID_MATRIX_NJ,ETA,cond)*J*Q1);
		Map<MatrixXd> Q2v (Q2.data(), NJOINTS*NJOINTS,1);
		res.Q2.data = vector<double> (Q2v.data(), Q2v.data() + Q2v.size());

		JL = J.block<3,NJOINTS>(0,0); // Extract linear part of the Jacobian matrix.
		if (firstStep) firstStep = false;
		else JLdot = (JL - JLold) / DT; // unless it's the first step, compute JLdot as a finite difference of linear Jacobian matrices.

		// Compute the non-optimized joint velocities.
		VectorXd qdot1; // will contain qdot computed according to closed loop IK first order.
		VectorXd qdot2; // will contain qdot computed according to closed loop IK second order.
		computeqdot(partialqdot,Q1,J,JL,JLdot,qdot,eta,rho,nu,v,w,a,qdot1,qdot2);
		JLold = JL; // update JLold
		
		// Fill response objects.
		res.qdot1.velocity = vector<double> (qdot1.data(), qdot1.data() + qdot1.size());
		res.qdot2.velocity = vector<double> (qdot2.data(), qdot2.data() + qdot2.size());
    }
    else { // if all subscription data is available but service call did not succeed
		readyJ = readyErr = readyVwa = readyqdot = false; // reset availability flags
    	ROS_ERROR("Call to safety service failed.");
    	return false;
    }

	// Timing
	/*clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "IK TOOK " << elapsed_secs << " SECS" << endl;*/

	return true; // call succeeded
}



/*! Main function of the node.
*/  
int main(int argc,char **argv) {
    ros::init(argc, argv, "ik_server"); // initialize node
    ros::NodeHandle n; // define node handle
    int queSize = 10;
    ros::Subscriber sub1 = n.subscribe("jacobian", queSize, ikCallbackJ); // subscribe to Jacobian
    ros::Subscriber sub2 = n.subscribe("errors", queSize, ikCallbackErr); // subscribe to errors
    ros::Subscriber sub3 = n.subscribe("tracking", queSize, ikCallbackVwa); // subscribe to tracking signals
    ros::Subscriber sub4 = n.subscribe("cmdtopic", queSize, ikCallbackqdot); // describe to weighter
    
    ros::ServiceServer service = n.advertiseService("ik", computeIKqdot); // activate IK service
    
    client = n.serviceClient<math_pkg::Safety>("safety"); // IK is client of Safety

    JLdot = MatrixXd::Zero(3,NJOINTS);

    ros::spin(); // keep the node alive
    return 0;
}