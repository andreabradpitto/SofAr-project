#include <algorithm>
#include <iostream>
#include "math_pkg/Cost.h"
#include "math_pkg/IK.h"
#include "math_pkg/Safety.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "utilities.h"

#include <ctime>

ros::ServiceClient clients[NUM_IK_SERVICES];
math_pkg::Cost costSrv;
//vector<double> qdots[NUM_IK_SOLUTIONS];


void getAllqdots(vector<double> qdots[], bool obtained[]) {
	bool costObtained;
	#pragma omp sections
	{
   		#pragma omp section
   		{
			   costObtained = clients[0].call(costSrv);
   		}
   		#pragma omp section
		{} // call to analytic

   		#pragma omp section
		{} // call to transp
	}
   	if (costObtained) {
		qdots[0] = costSrv.response.qdot1opt1.velocity;
		qdots[1] = costSrv.response.qdot1opt2.velocity;
		qdots[2] = costSrv.response.qdot2opt1.velocity;
		qdots[3] = costSrv.response.qdot2opt2.velocity;
	}
	else {
		// ROS ERROR... domani
	}
	for (int i = 0; i < NUM_OPTIMIZED_SOLUTIONS; i++) {
		obtained[i] = costObtained;
	}
}


sensor_msgs::JointState computeWeightedqdot() {
	vector<double> finalqdot(NJOINTS,0);
	sensor_msgs::JointState finalqdotState;
	bool obt[NUM_IK_SOLUTIONS];
	vector<double> qdots[NUM_IK_SOLUTIONS];
	getAllqdots(qdots,obt);
	double weight;
	for (int i = 0; i < NUM_IK_SOLUTIONS; i++) {
		if (obt[i]) {
			weight = WEIGHTS[i];
			transform(qdots[i].begin(), qdots[i].end(), qdots[i].begin(), [&weight](double& c){return c*weight;});
			transform(qdots[i].begin(), qdots[i].end(), finalqdot.begin(), finalqdot.begin(), std::plus<double>());
		}
	}
	finalqdotState.velocity = finalqdot;
	return finalqdotState;
}


int main(int argc,char **argv) {
    ros::init(argc, argv, "weighter");
    ros::NodeHandle n;

    int queSize = 10;
    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("jointvel", queSize);
    ros::Rate loopRate(1/DT);

    clients[0] = n.serviceClient<math_pkg::Cost>("cost");
    //clients[1] = n.serviceClient<math_pkg::IK>("");
    //clients[2] = n.serviceClient<math_pkg::IK>("");

    cout << "WEIGHTER WILL NOW PROCEED TO WEIGH" << endl;

    while (ros::ok()) {
		clock_t begin = clock();
    	pub.publish(computeWeightedqdot());
		clock_t end = clock();
		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		cout << "TOOK " << elapsed_secs << " SECS" << endl;
    	ros::spinOnce();
    	loopRate.sleep();
    }

    return 0;
}