/*! \file */

#include <fstream>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>
#include <vector>
#include "cmath"
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "math_pkg/Cost.h"
#include "Eigen/QR"

using namespace Eigen;
using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

/*! Simulation timestep.*/
#define DT 0.05
/*! Auxiliary parameter for pseudoinversion.*/
#define ETA 5
/*! Majorant of joint angles.*/
#define JOINTS_MAJ 3.8
/*! Soft margin of joint angles.*/
#define JOINTS_MARGIN 0.1
/*! Max absolute value of correction pole for joint angles.*/
#define JOINTS_MAXPOLE 3
/*! Linear positional gain for tracking in CLIK1.*/
#define Kpp 10
/*! Linear positional gain for tracking in CLIK2.*/
#define Kp 400
/*! Linear velocity gain for tracking.*/
#define Kv 40
/*! Rotational gain for tracking.*/
#define Krot 10
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
#define VEL_MAJ 20
/*! Soft margin of joint  velocities.*/
#define VEL_MARGIN 0.1
/*! Max absolute value of correction pole for joint velocity.*/
#define VEL_MAXPOLE 20
/*! Abs value of the correction pole for safety task.*/
#define CORRPOLE 70

/*! Min joint angles.*/
const double QMIN[] = {-1.6817,-2.1268,-3.0343,-0.0300,-3.0396,-1.5508,-3.0396};
/*! Max joint angles.*/
const double QMAX[] = {1.6817,1.0272,3.0343,2.5829,3.0378,2.0744,3.0378};
/*! Min joint velocities.*/
const double QDOTMIN[] = {-3,-3,-3,-3,-3,-3,-3};
/*! Max joint velocities.*/
const double QDOTMAX[] = {3,3,3,3,3,3,3};
/*! Initial joint angles.*/
const double QINIT[] = {0,0,0,0,0,0,0};
/*! Constant used in Gaussian computation for pseudoinversion.*/
const double b = -log(0.5)/0.000000001;
/*! Identity matrix of size NJOINTS.*/
const MatrixXd ID_MATRIX_NJ = MatrixXd::Identity(NJOINTS,NJOINTS);
/*! Identity matrix of size 6.*/
const MatrixXd ID_MATRIX_SPACE_DOFS = MatrixXd::Identity(SPACE_DOFS,SPACE_DOFS);
/*! Vector of ones of size NJOINTS.*/
const VectorXd ONES_VEC_NJ = VectorXd::Constant(NJOINTS,1);
/*! Vector of preferred joint velocities.*/
const VectorXd QDOT_FAV((VectorXd(NJOINTS) << 0, 0, 0, 0, 0, 0, 0).finished());
/*! Zero matrix of size NJOINTS.*/
const MatrixXd ZERO_MATRIX_NJ = MatrixXd::Zero(NJOINTS,NJOINTS);
 

bool stay_still = false;

ofstream xedot1file("xedot1.txt");
ofstream qdot1file("qdotCLIK1.txt");
ofstream tempProduct2File("tempProduct2.txt");
ofstream Jfile("J.txt");
ofstream verifyik("verifyik.txt");
ofstream pinvNonAux2file("pinvNonAux2.txt");
ofstream pinvAux2file("pinvAux2.txt");
ofstream vgfile("vg.txt");
ofstream wgfile("wg.txt");
ofstream agfile("ag.txt");

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
    \param corrPole Absolute value of correction pole.
    \param currentPole Current correction pole for x, if any, 0 otherwise.
    \param isJoint True if x is a joint angle, false otherwise.
    \param rdot Reference to task element derivative for quantity x.
    \param Adiag Reference to activation value for quantity x.
    \return true if no correction needed, false otherwise.
*/
bool jointConstr(double x,const double xmin,const double xmax,
	const double mrgn,const double corrPole,
	double &currentPole,const bool isJoint,double &rdot, double &Adiag) {
    bool ok = false;

    // Temp
    double cpole;
    if (isJoint) cpole = 70;
    else cpole = 50;

	if (x > xmax - mrgn) {
        //cout << "too high" << x << ">" << xmax - mrgn << ", is joint = " << isJoint << endl;
		if (currentPole == 0) currentPole = cpole;
		if (isJoint) rdot = currentPole * (-x + xmax - mrgn);
		else rdot = currentPole * (-x + xmax - mrgn) * DT + x;
		if (x > xmax) Adiag = 1;
		else Adiag = cos_sigmoid(x,xmax,mrgn);
	}
	else if (x < xmin + mrgn) {
        //cout << "too low" << x << "<" << xmin + mrgn << ", is joint = " << isJoint << endl;
		if (currentPole == 0) currentPole = cpole;
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
    MatrixXd toSVD = XT*A*X + 5*(idMinusQ.transpose()*idMinusQ);

    JacobiSVD<MatrixXd> svd(toSVD, ComputeThinU | ComputeThinV);
    VectorXd sv = svd.singularValues();
    /*cout << "~~~~~~~~~~~" << endl;
    cout << "XT*A*X = " << XT*A*X << endl;
    cout << "eta*... = " << eta*(idMinusQ.transpose()*idMinusQ) << endl;
    cout << "X = " << X << endl;*/
    cout << "SV = " << sv << endl;

    for (int i = 0; i < sv.size(); i++) {
        if (abs(sv(i)) > 1e-8) {
  		    bel = exp(-b*sv(i)*sv(i));
    	    //sv(i) = sv(i)/(sv(i)*sv(i) + bel*bel); // singular values too big are replaced by small values
            sv(i) = bel;
        }
        else break;
    }
    cout << "tosvd = " << toSVD << endl;
    cout << "P:" << sv << endl;
    cout << "V = " << svd.matrixV() << endl;
    cout << "VTPV" << svd.matrixV().transpose() * sv.asDiagonal() * svd.matrixV() << endl;
    //cout << "XT * A * A" << XT * A * A << endl;
    return (toSVD + svd.matrixV().transpose() * sv.asDiagonal() * svd.matrixV()).completeOrthogonalDecomposition().pseudoInverse() * XT * A * A;
    //return svd.matrixV().transpose() * (sv.asDiagonal()) * (svd.matrixU()) * XT * A * A;
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
    VectorXd ve1 = v + Kpp*eta;
    vgfile << v << endl << endl;
    wgfile << w << endl << endl;
    agfile << a << endl << endl;
    VectorXd ve2 = DT*(a - JLdot*qdot + Kv*etadot + Kp*eta) + JL*qdot;
    VectorXd xedot1 = VectorXd(6);
    VectorXd xedot2 = VectorXd(6);
    xedot1 << ve1,w+Krot*rho;
    xedot2 << ve2,w+Krot*rho;
    xedot1file <<  xedot1 << endl << endl;
    MatrixXd JTimesQ1 = J*Q1;
    MatrixXd pinvAux = regPinv(JTimesQ1,ID_MATRIX_SPACE_DOFS,Q1,ETA);
    MatrixXd pinvQZero = regPinv(JTimesQ1,ID_MATRIX_SPACE_DOFS,ID_MATRIX_NJ,ETA);
    pinvAux2file << pinvAux << endl << endl;
    pinvNonAux2file << pinvQZero << endl << endl;
    MatrixXd W2 = JTimesQ1*pinvAux;
    /*
    cout << "Q2=" << Q1 << endl;
    cout << "JTimesQ1=" << JTimesQ1 << endl;
    cout << "pinvQZero=" << pinvQZero << endl;
    cout << "W2=" << W2 << endl;
    */
    MatrixXd tempProduct1 = Q1*pinvQZero*W2;
    MatrixXd tempProduct2 = J*partialqdot;
    qdot1 = partialqdot + tempProduct1 * (xedot1 - tempProduct2);
    qdot1file <<  qdot1 << endl << endl;
    Jfile <<  J << endl << endl;
    tempProduct2File <<  tempProduct2 << endl << endl;
    qdot2 = partialqdot + tempProduct1 * (xedot2 - tempProduct2);
	verifyik << J*qdot1 - xedot1 << endl << endl;
    //cout << "qdot1 = " << qdot1;
    //cout << "qdot2 = " << qdot2;
}


/*! Function that prints a std::vector of doubles, used for debug.
    \param v A std::vector of doubles.
*/
void printVectord(vector<double> v, char * name) {
    cout << name << ":" << endl;
    for (int i = 0; i < v.size(); i++) {
        cout << v[i] << ",";
    }
    cout << endl;
}

/*! Function that prints an array of doubles, used for debug.
    \param v A std::vector of doubles.
*/
void printArrayd(double v[], int size, char name[]) {
    cout << name << ":" << endl;
    for (int i = 0; i < size; i++) {
        cout << v[i] << ",";
    }
    cout << endl;
}

/*! Function that saturates joint velocities.
    \param qdots Vector to be saturated
*/
void saturate (vector<double> &qdots) {
    for (short i = 0; i < NJOINTS; i++) {
        if (qdots[i] > QDOTMAX[i]) qdots[i] = QDOTMAX[i];
        else if (qdots[i] < QDOTMIN[i]) qdots[i] = QDOTMIN[i];
    }
}