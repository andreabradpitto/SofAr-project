#include <algorithm>
#include <iostream>
#include "math_pkg/IK.h"
#include "math_pkg/Safety.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "utilities.h"

#include <ctime>

bool firstStep = true;
bool readyJ,readyErr,readyVwa,readyqdot;
MatrixXd J,JL,JLdot,JLold;
VectorXd eta,rho,etadot,rhodot,v,w,a,qdot;
ros::ServiceClient client;
math_pkg::Safety safeSrv;


void ikCallbackJ(const std_msgs::Float64MultiArray &msg)
{
	vector<double> rcvdJ = msg.data;
	J = Map<MatrixXd>(rcvdJ.data(),NJOINTS,6).transpose();
	readyJ = true;
}

void ikCallbackErr(const std_msgs::Float64MultiArray &msg)
{
	vector<double> rcvdErr = msg.data;
	eta = Map<VectorXd>(rcvdErr.data(),3);
	rho = Map<VectorXd>(rcvdErr.data()+3,3);
	etadot = Map<VectorXd>(rcvdErr.data()+6,3);
	rhodot = Map<VectorXd>(rcvdErr.data()+9,3);
	readyErr = true;
}

void ikCallbackVwa(const std_msgs::Float64MultiArray &msg)
{
	vector<double> rcvdVwa = msg.data;
	v = Map<VectorXd>(rcvdVwa.data(),3);
	w = Map<VectorXd>(rcvdVwa.data()+3,3);
	a = Map<VectorXd>(rcvdVwa.data()+6,3);
	readyVwa = true;
}

void ikCallbackqdot(const sensor_msgs::JointState &msg)
{
	vector<double> rcvdqdot = msg.velocity;
	qdot = Map<VectorXd>(rcvdqdot.data(),NJOINTS);
	readyqdot = true;
}


bool computeIKqdot(math_pkg::IK::Request  &req, math_pkg::IK::Response &res) {
    //clock_t begin = clock();
	if (!(readyJ && readyErr && readyVwa && readyqdot)) {
		readyJ = readyErr = readyVwa = readyqdot = false;
    	ROS_ERROR("ik service could not run: missing data.");
    	return false;
	}
    if (client.call(safeSrv)) {
		readyJ = readyErr = readyVwa = readyqdot = false;
    	VectorXd partialqdot = Map<VectorXd>(safeSrv.response.qdot.velocity.data(),NJOINTS);
    	MatrixXd Q2 = Map<MatrixXd>(safeSrv.response.Q2.data.data(),NJOINTS,NJOINTS);
		JL = J.block<3,NJOINTS>(3,0);
		if (firstStep) firstStep = false;
		else JLdot = (JL - JLold) / DT;
		Ik ik = computeqdot(partialqdot,Q2,J,JL,JLdot,qdot,eta,rho,etadot,rhodot,
			v,w,a);
		JLold = JL;
		VectorXd qdot1 = ik.qdot1;
		VectorXd qdot2 = ik.qdot2;
		res.qdot1.velocity = vector<double> (qdot1.data(), qdot1.data() + qdot1.size());
		res.qdot2.velocity = vector<double> (qdot2.data(), qdot2.data() + qdot2.size());
    }
    else {
		readyJ = readyErr = readyVwa = readyqdot = false;
    	ROS_ERROR("Call to safety service failed.");
    	return false;
    }
	/*clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "IK TOOK " << elapsed_secs << " SECS" << endl;*/
	return true;
}


int main(int argc,char **argv) {
    ros::init(argc, argv, "ik_server");
    ros::NodeHandle n;
    int queSize = 10;
    ros::Subscriber sub1 = n.subscribe("Jac", queSize, ikCallbackJ);
    ros::Subscriber sub2 = n.subscribe("computed_errors", queSize, ikCallbackErr);
    ros::Subscriber sub3 = n.subscribe("v_w_a", queSize, ikCallbackVwa);
    ros::Subscriber sub4 = n.subscribe("joint_vel", queSize, ikCallbackqdot);
    
    ros::ServiceServer service = n.advertiseService("ik", computeIKqdot);
    
    client = n.serviceClient<math_pkg::Safety>("safety");
    
    JLdot = MatrixXd::Zero(3,NJOINTS);
    cout << "IK WILL NOW PROCEED TO SPIN" << endl;
    ros::spin();
    return 0;
}