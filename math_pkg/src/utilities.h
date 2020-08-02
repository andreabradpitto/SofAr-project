/*! \file */

#include <condition_variable>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>
#include <vector>
#include "cmath"
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "math_pkg/Cost.h"

using namespace Eigen;
using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

/*! Simulation timestep.*/
#define DT 0.01
/*! Auxiliary parameter for pseudoinversion.*/
#define ETA 0
/*! Majorant of joint angles.*/
#define JOINTS_MAJ 3
/*! Soft margin of joint angles.*/
#define JOINTS_MARGIN 0.2
/*! Max absolute value of correction pole for joint angles.*/
#define JOINTS_MAXPOLE 1
/*! Linear positional gain for tracking.*/
#define Kp 1
/*! Linear velocity gain for tracking.*/
#define Kv 25
/*! Rotational gain for tracking.*/
#define Krot 25
/*! Number of robot joint.*/
#define NJOINTS 7
/*! Number of invkin services.*/
#define NUM_IK_SERVICES 3
/*! Number of invkin solutions (J transpose + analytical + 4 from Cost service).*/
#define NUM_IK_SOLUTIONS 6
/*! Number of optimized invkin solutions (the 4 from Cost service).*/
#define NUM_OPTIMIZED_SOLUTIONS 4
/*! Regularization threshold for pseudoinversion.*/
#define REG_THR 0.1
/*! Spatial dofs of the robot.*/
#define SPACE_DOFS 6
/*! Majorant of joint velocities.*/
#define VEL_MAJ 3
/*! Soft margin of joint  velocities.*/
#define VEL_MARGIN 0.2
/*! Max absolute value of correction pole for joint velocity.*/
#define VEL_MAXPOLE 20

/*! Min joint angles.*/
const double QMIN[] = {-5555, 6, 7, 8, 3, 4, 5};
/*! Max joint angles.*/
const double QMAX[] = {5000, 6, 7, 8, 3, 4, 5};
/*! Min joint velocities.*/
const double QDOTMIN[] = {-2.4609, -0.5760, -3.0194, -0.0524, -3.0543, -1.5708, -3.0543};
/*! Max joint velocities.*/
const double QDOTMAX[] = {0.8901, 2.6180, 3.0194, 2.6180, 3.0543, 2.0944, 3.0543};
/*! Initial joint angles.*/
const double QINIT[] = {5, 6, 7, 8, 3, 4, 5};
/*! Weights for invkin solutions (sum must be 1).*/
const double WEIGHTS[] = {0,0,1,0,0,0};
/*! Constant used in Gaussian computation for pseudoinversion.*/
const double b = -log(0.5)/0.01;
/*! Identity matrix of size NJOINTS.*/
const MatrixXd ID_MATRIX_NJ = MatrixXd::Identity(NJOINTS,NJOINTS);
/*! Identity matrix of size 6.*/
const MatrixXd ID_MATRIX_SPACE_DOFS = MatrixXd::Identity(SPACE_DOFS,SPACE_DOFS);
/*! Vector of ones of size NJOINTS.*/
const VectorXd ONES_VEC_NJ = VectorXd::Constant(NJOINTS,1);
/*! Vector of preferred joint velocities.*/
const VectorXd QDOT_FAV((VectorXd(NJOINTS) << 1, 2, 3, 4, 5, 6, 7).finished());
/*! Zero matrix of size NJOINTS.*/
const MatrixXd ZERO_MATRIX_NJ = MatrixXd::Zero(NJOINTS,NJOINTS);
 

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
    \param maxPole Max absolute value of the correction pole for x.
    \param maj A majorant of x.
    \param currentPole Current correction pole for x, if any, 0 otherwise.
    \param isJoint True if x is a joint angle, false otherwise.
    \param rdot Reference to task element derivative for quantity x.
    \param Adiag Reference to activation value for quantity x.
    \return true if no correction needed, false otherwise.
*/
bool jointConstr(double x,const double xmin,const double xmax,
	const double mrgn,const double maxPole,const double maj,
	double &currentPole,const bool isJoint,double &rdot, double &Adiag) {
    bool ok = false;
	if (x > xmax - mrgn) {
		if (currentPole == 0) currentPole = min(maxPole,float(maj)/abs(x-xmax));
		if (isJoint) rdot = currentPole * (-x + xmax - mrgn);
		else rdot = currentPole * (-x + xmax - mrgn) * DT + x;
		if (x > xmax) Adiag = 1;
		else Adiag = cos_sigmoid(x,xmax,mrgn);
	}
	else if (x < xmin + mrgn) {
		if (currentPole == 0) currentPole = min(maxPole,float(maj)/abs(x-xmin));
		if (isJoint) rdot = currentPole * (-x + xmin + mrgn);
		else rdot = currentPole * (-x + xmin + mrgn) * DT + x;
		if (x < xmin) Adiag = 1;
		else Adiag = cos_sigmoid(x,xmin,mrgn);
	}
	else {
		Adiag = 0;
		rdot = 0;
		ok = true;
        currentPole = 0;
	}
    return ok;
}


/*! Function that computes regularized pseudoinverse of matrix X.
    \param X Matrix to be pseudoinverted.
    \param A Activation matrix.
    \param Q Auxiliary matrix.
    \param eta Auxiliary parameter.
    \return the regularized pseudoinverse of X.
*/
MatrixXd regPinv(MatrixXd X,MatrixXd A,MatrixXd Q,double eta) {
	// Adiag has size NJOINTS
    double bel;
    MatrixXd XT = X.transpose();
    MatrixXd idMinusQ = ID_MATRIX_NJ - Q;
    MatrixXd toSVD = XT*A*X + eta*(idMinusQ*idMinusQ.transpose());

    JacobiSVD<MatrixXd> svd(toSVD, ComputeThinU | ComputeThinV);
    VectorXd sv = svd.singularValues();

    for (int i = 0; i < NJOINTS; i++) {
  		bel = exp(-b*sv(i)*sv(i));
    	sv(i) = sv(i)/(sv(i)*sv(i) + bel*bel); // singular values too big are replaced by small values
    }

    return svd.matrixV().transpose() * (sv.asDiagonal()) * (svd.matrixU().transpose()) * XT * A * A;
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
    VectorXd ve1 = v + Kp*eta;
    VectorXd ve2 = DT*(a - JLdot*qdot + Kv*etadot + Kp*eta) + JL*qdot;
    VectorXd xedot1 = VectorXd(6);
    VectorXd xedot2 = VectorXd(6);
    xedot1 << ve1,Krot*rho;
    xedot2 << ve2,Krot*rho;
    MatrixXd JTimesQ1 = J*Q1;
    MatrixXd pinvAux = regPinv(JTimesQ1,ID_MATRIX_SPACE_DOFS,Q1,ETA);
    MatrixXd pinvQZero = regPinv(JTimesQ1,ID_MATRIX_SPACE_DOFS,ZERO_MATRIX_NJ,ETA);
    MatrixXd W2 = JTimesQ1*pinvAux;
    MatrixXd tempProduct1 = Q1*pinvQZero*W2;
    MatrixXd tempProduct2 = J*partialqdot;
    qdot1 = partialqdot + tempProduct1 * (xedot1 - tempProduct2);
    qdot2 = partialqdot + tempProduct1 * (xedot2 - tempProduct2);
}


/*! Function that prints a std::vector of doubles, used for debug.
    \param v A std::vector of doubles.
*/
void printVectord(vector<double> v) {
    for (int i = 0; i < v.size(); i++) {
        cout << v[i] << "," << endl;
    }
}