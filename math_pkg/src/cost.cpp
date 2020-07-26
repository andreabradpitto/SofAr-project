#include <algorithm>
#include <iostream>
#include "math_pkg/Cost.h"
#include "math_pkg/IK.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "utilities.h"

#include <ctime>

bool readyJ;
MatrixXd J;
ros::ServiceClient client;
math_pkg::IK ikSrv;


void costCallbackJ(const std_msgs::Float64MultiArray &msg)
{
	vector<double> rcvdJ = msg.data;
	J = Map<MatrixXd>(rcvdJ.data(),NJOINTS,6).transpose();
    readyJ = true;
}


bool computeOptqdot(math_pkg::Cost::Request  &req, math_pkg::Cost::Response &res) {
    //clock_t begin = clock();
    if (!readyJ) {
        readyJ = false;
    	ROS_ERROR("cost service could not run: missing data.");
    	return false;
    }
    if (client.call(ikSrv)) {
        readyJ = false;
    	VectorXd qdot1 = Map<VectorXd>(ikSrv.response.qdot1.velocity.data(),NJOINTS);
    	VectorXd qdot2 = Map<VectorXd>(ikSrv.response.qdot2.velocity.data(),NJOINTS);
   	 	computeCostResponse(J,qdot1,qdot2,res);
   	}
    else {
        readyJ = false;
    	ROS_ERROR("Call to ik service failed.");
    	return false;
    }
	/*clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "CST TOOK " << elapsed_secs << " SECS" << endl;*/
	return true;
}


int main(int argc,char **argv) {
    ros::init(argc, argv, "cost_server");
    ros::NodeHandle n;

    int queSize = 10;
    ros::Subscriber sub = n.subscribe("Jac", queSize, costCallbackJ);

    ros::ServiceServer service = n.advertiseService("cost", computeOptqdot);
    
    client = n.serviceClient<math_pkg::IK>("ik");

    cout << "COST WILL NOW PROCEED TO SPIN" << endl;
    ros::spin();
    return 0;
}