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
bool readyJ;

/*! Current Jacobian matrix.*/
MatrixXd J;

/*! Client object needed to perform service calls.*/
ros::ServiceClient client;

/*! IK server object.*/
math_pkg::IK ikSrv;



/*! Callback function for Jacobian matrix.
    \param msg The received Jacobian matrix.
*/
void costCallbackJ(const std_msgs::Float64MultiArray &msg)
{
	vector<double> rcvdJ = msg.data;
	J = Map<MatrixXd>(rcvdJ.data(),NJOINTS,6).transpose();
    readyJ = true;
}



/*! Function that computes the optimized velocity vectors.
    \param J Current Jacobian matrix.
	\param qdot1 Joint velocity vector computed with closed loop IK of order 1.
	\param qdot2 Joint velocity vector computed with closed loop IK of order 2.
    \param res Response to client of cost service.
*/
void computeCostResponse(MatrixXd J,VectorXd qdot1,VectorXd qdot2,math_pkg::Cost::Response &res) {
    VectorXd qdot1opt1,qdot1opt2,qdot2opt1,qdot2opt2; // initialization

    // Compute matrix G = joint kernel matrix.
    MatrixXd G = ID_MATRIX_NJ - (regPinv(J,ID_MATRIX_SPACE_DOFS,ID_MATRIX_NJ,ETA)) * J;
    MatrixXd GTimesGSharp = G*regPinv(G,ID_MATRIX_NJ,ID_MATRIX_NJ,ETA);
    VectorXd z1 = G*ONES_VEC_NJ; // adjust here

    // Each optimized velocity vector is given by: non-optimized vector + G * z, where z is a NJOINTSx1 vector.
    qdot1opt1 = qdot1 + z1;
    qdot1opt2 = qdot1 + GTimesGSharp*(QDOT_FAV - qdot1);
    qdot2opt1 = qdot2 + z1;
    qdot1opt2 = qdot1 + GTimesGSharp*(QDOT_FAV - qdot2);

    // Fill the response object.
    res.qdot1opt1.velocity = vector<double> (qdot1opt1.data(),qdot1opt1.data()+qdot1opt1.size());
    res.qdot1opt2.velocity = vector<double> (qdot1opt2.data(),qdot1opt2.data()+qdot1opt2.size());
    res.qdot2opt1.velocity = vector<double> (qdot2opt1.data(),qdot2opt1.data()+qdot2opt1.size());
    res.qdot2opt2.velocity = vector<double> (qdot2opt2.data(),qdot2opt2.data()+qdot2opt2.size());
}



/*! Service function for the Safety service, which computes the optimized joint velocities according to 2 optimization criteria: closeness to a preferred positional
configuration and closeness to a preferred velocity vector.
    \param req Empty.
	\param res Four optimized joint velocities, respectively computed from IK 1 with cost function 1 and 2, and from IK 2 again with both cost functions.
    \return true if client-service call succeeded, false otherwise.
*/
bool computeOptqdot(math_pkg::Cost::Request  &req, math_pkg::Cost::Response &res) {
    //clock_t begin = clock();
    if (!readyJ) { // Jacobian not available
        readyJ = false; // reset availability flag
    	ROS_ERROR("cost service could not run: missing data.");
    	return false;
    }
    if (client.call(ikSrv)) { // if Jacobian is available and the service call succeeded
        readyJ = false; // reset availability flag

		// Map the non-optimized vectors returned by the call into Eigen library objects.
    	VectorXd qdot1 = Map<VectorXd>(ikSrv.response.qdot1.velocity.data(),NJOINTS);
    	VectorXd qdot2 = Map<VectorXd>(ikSrv.response.qdot2.velocity.data(),NJOINTS);

        // Compute optimized vectors and fill the response object.
   	 	computeCostResponse(J,qdot1,qdot2,res);
   	}
    else { // if Jacobian is available but the service call did not succeed
        readyJ = false; // reset availability flag
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
    ros::Subscriber sub = n.subscribe("jacobian", queSize, costCallbackJ); // subscribe for Jacobian

    ros::ServiceServer service = n.advertiseService("cost", computeOptqdot); // activate Cost service
    
    client = n.serviceClient<math_pkg::IK>("ik"); // Cost is a client of IK

    cout << "COST WILL NOW PROCEEDTO SPIN" << endl;
    ros::spin();
    return 0;
}