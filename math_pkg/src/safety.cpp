#include <algorithm>
#include <iostream>
#include <thread>
#include "math_pkg/Safety.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "utilities.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ctime>

bool angleOk,readyq,readyqdot;
double currentAnglePole[NJOINTS];
double currentVelPole[NJOINTS];
double q[NJOINTS];
double qdot[NJOINTS];

void safetyCallback(const JointStateConstPtr& msgq,
const JointStateConstPtr& msgqdot) {
	vector<double> rcvd_q = msgq->position;
	vector<double> rcvd_qdot = msgqdot->velocity;
	std::copy(rcvd_q.begin(), rcvd_q.end(), q);
	std::copy(rcvd_qdot.begin(), rcvd_qdot.end(), q);
	readyq = readyqdot = true;
}

void safetyCallbackq(const sensor_msgs::JointState &msg) { 
	vector<double> rcvd_q = msg.position;
	std::copy(rcvd_q.begin(), rcvd_q.end(), q);
	readyq = true;
}

void safetyCallbackqdot(const sensor_msgs::JointState &msg) {
	vector<double> rcvd_qdot = msg.velocity;
	std::copy(rcvd_qdot.begin(), rcvd_qdot.end(), q);
	readyqdot = true;
}

inline double cos_sigmoid(double x,double y,double mrgn) {
	return (.5 + .5 * cos((x - y) * M_PI/mrgn));
}


Safety jointConstr(double x,const double xmin,const double xmax,
	const double mrgn,const double maxPole,const double maj,
	double &currentPole,const double DT,bool isJoint,bool &angleOk) {
	double rdot,Adiag;
	if (x > xmax - mrgn) {
		if (currentPole == 0)
			currentPole = min(maxPole,float(maj)/abs(x-xmax));
		if (isJoint)
			rdot = currentPole * (-x + xmax - mrgn);
		else
			rdot = currentPole * (-x + xmax - mrgn) * DT + x;
		if (x > xmax)
			Adiag = 1;
		else
			Adiag = cos_sigmoid(x,xmax,mrgn);
	}
	else if (x < xmin + mrgn) {
		if (currentPole == 0)
			currentPole = min(maxPole,float(maj)/abs(x-xmax));
		if (isJoint)
			rdot = currentPole * (-x + xmin + mrgn);
		else
			rdot = currentPole * (-x + xmin + mrgn) * DT + x;
		if (x < xmin)
			Adiag = 1;
		else
			Adiag = cos_sigmoid(x,xmin,mrgn);
	}
	else {
		Adiag = 0;
		rdot = 0;
		angleOk = true;
	}
	return Safety(rdot,Adiag);
}


bool computePartialqdot(math_pkg::Safety::Request  &req, math_pkg::Safety::Response &res) {
	//clock_t begin = clock();
	if (!(readyq && readyqdot)) {
    	ROS_ERROR("safety service could not run: missing data.");
		return false;
	}
	readyq = readyqdot = false;
	VectorXd partial_qdot(NJOINTS);VectorXd rdot(NJOINTS);VectorXd Adiag(NJOINTS);
	res.qdot.velocity.clear();

	for(short i = 0; i<NJOINTS; i++) {
		angleOk = false;
	    Safety safety = jointConstr(q[i],QMIN[i],QMAX[i],JOINTS_MARGIN,JOINTS_MAXPOLE,JOINTS_MAJ,currentAnglePole[i],0,true,angleOk);
	    rdot(i) = safety.rdot;
	    Adiag(i) = safety.Adiag;
		if (angleOk) {
			Safety safety = jointConstr(qdot[i],-QDOTMAX[i],QDOTMAX[i],VEL_MARGIN,VEL_MAXPOLE,VEL_MAJ,currentVelPole[i],0,false,angleOk);
	    	rdot(i) = safety.rdot;
	    	Adiag(i) = safety.Adiag;
		}
	}
    MatrixXd A = Adiag.asDiagonal();
	MatrixXd pinvJ = regPinv(ID_MATRIX_NJ,A,ZERO_MATRIX_NJ,ETA);
	MatrixXd Q2 = ID_MATRIX_NJ - pinvJ;
	Map<MatrixXd> Q2v (Q2.data(), NJOINTS*NJOINTS,1);
	
	partial_qdot = pinvJ * rdot;
	res.qdot.velocity = vector<double> (partial_qdot.data(), partial_qdot.data() + partial_qdot.size());
	res.Q2.data = vector<double> (Q2v.data(), Q2v.data() + Q2v.size());
	/*clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "SFT TOOK " << elapsed_secs << " SECS" << endl;*/
	return true;
}

int main(int argc,char **argv) {
    ros::init(argc, argv, "safety_server");
    ros::NodeHandle n;
    int queSize = 10;
	
	// With sync:
	/*Subscriber<JointState> subq(n,"cmdtopic",queSize);
	Subscriber<JointState> subqdot(n,"joint_vel",queSize);
	typedef sync_policies::ApproximateTime<JointState, JointState> MySyncPolicy;
  	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subq, subqdot);
  	sync.registerCallback(boost::bind(&safetyCallback, _1, _2));*/
	
	// Without sync:
	ros::Subscriber sub1 = n.subscribe("cmdtopic", queSize, safetyCallbackq);
    ros::Subscriber sub2 = n.subscribe("joint_vel", queSize, safetyCallbackqdot);
    
	ros::ServiceServer service = n.advertiseService("safety", computePartialqdot);
    cout << "SAFETY WILL NOW PROCEED TO SPIN" << endl;
    ros::spin();
    return 0;
}