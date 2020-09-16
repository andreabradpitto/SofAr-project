/*! \file */

#include <fstream>
#include <iostream>
#include <vector>
#include "cmath"
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "math_pkg/Cost.h"

using namespace Eigen;
using namespace std;
using namespace sensor_msgs;

/*! Simulation timestep.*/
#define DT 0.01
/*! Auxiliary parameter for pseudoinversion.*/
#define ETA 5
/*! Number of robot joint.*/
#define NJOINTS 7
/*! Number of invkin services.*/
#define NUM_IK_SERVICES 3
/*! Number of invkin solutions (J transpose + analytical + 4 from Cost service).*/
#define NUM_IK_SOLUTIONS 6
/*! Number of optimized invkin solutions (the 4 from Cost service).*/
#define NUM_OPTIMIZED_SOLUTIONS 4
/*! Spatial dofs of the robot.*/
#define SPACE_DOFS 6
/*! Min joint angles.*/
const double QMIN[] = {-1.6817,-2.1268,-3.0343,-0.3,-3.0396,-1.5508,-3.0396};
/*! Max joint angles.*/
const double QMAX[] = {1.6817,1.0272,3.0343,2.5829,3.0378,2.0744,3.0378};
/*! Min joint velocities.*/
const double QDOTMIN[] = {-1,-1,-1,-1,-1,-1,-1};
/*! Max joint velocities.*/
const double QDOTMAX[] = {1,1,1,1,1,1,1};
/*! Initial joint angles.*/
const double QINIT[] = {0,0,0,0,0,0,0};
/*! Constant used in Gaussian computation for pseudoinversion.*/
const double bellConstantGX = -log(0.5)/0.000001;
/*! Identity matrix of size NJOINTS.*/
const MatrixXd ID_MATRIX_NJ = MatrixXd::Identity(NJOINTS,NJOINTS);
/*! Identity matrix of size 6.*/
const MatrixXd ID_MATRIX_SPACE_DOFS = MatrixXd::Identity(SPACE_DOFS,SPACE_DOFS);
/*! Vector of ones of size NJOINTS.*/
const VectorXd ONES_VEC_NJ = VectorXd::Constant(NJOINTS,1);
/*! Zero matrix of size NJOINTS.*/
const MatrixXd ZERO_MATRIX_NJ = MatrixXd::Zero(NJOINTS,NJOINTS);
/*! Sequence number used for synchronization.*/
int seq = 0;
/*! Becomes true when error is zero.*/
bool stay_still = false;


/*! Pseudoinverse of matrix A. Since the pseudoinverse routine of the Eigen library is unstable (see 
https://eigen.tuxfamily.org/dox/classEigen_1_1CompleteOrthogonalDecomposition.html), this function was coded following 
MATLAB's pinv's source code.
    \param A matrix to be pseudoinverted
    \return the pseudoinverse of A.
*/
MatrixXd mypinv(MatrixXd A) {
    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
	VectorXd s2 = svd.singularValues();
	double tol = 1e-16; // threshold for singular values
    int cnt = 0;
	int s2sz = s2.size();
	for (int i = 0;	i < s2sz; i++) {
		if (s2(i) > tol) cnt++;
		else break;
	}

    // Throw away portions of U, V related to null singular values of A.
	VectorXd s2rev = s2.head(cnt);
	MatrixXd U = svd.matrixU();
	MatrixXd Urev = U.block(0,0,U.rows(),cnt);
	MatrixXd V = svd.matrixV();
	MatrixXd Vrev = V.block(0,0,V.rows(),cnt);

    // Compute pseudoinverse.
	s2rev = s2rev.cwiseInverse();
	MatrixXd s2diag = s2rev.asDiagonal();
	MatrixXd pinvA = Vrev * s2diag * Urev.transpose();
	
	return pinvA;
}


/*! Function that computes generalized regularized pseudoinverse of matrix X according  to the paper "A Novel Practical Technique to Integrate 
    Inequality Control Objectives and Task Transitions in Priority Based Control" by Casalino & Simetti, p. 20, sec. 4.3.
    \param X Matrix to be pseudoinverted.
    \param A Activation matrix.
    \param Q Auxiliary matrix.
    \param eta Auxiliary parameter.
    \return the regularized pseudoinverse of X.
*/
MatrixXd regPinv(MatrixXd X,MatrixXd A,MatrixXd Q,double eta,double &cond) {
	// Adiag has size NJOINTS
    MatrixXd XT = X.transpose();
    MatrixXd idMinusQ = ID_MATRIX_NJ - Q;
    MatrixXd toSVD = XT*A*X + eta*(idMinusQ.transpose()*idMinusQ);

    JacobiSVD<MatrixXd> svd(toSVD, ComputeThinU | ComputeThinV); // compute SVD
    VectorXd sv = svd.singularValues();
    int svsz = sv.size(); // number of singular values

    for (int i = 0; i < svsz; i++) {
        if (abs(sv(i)) > 1e-8) sv(i) = exp(-bellConstantGX*sv(i)*sv(i)); // bell-shaped regularization function
        else break;
    }
    cond = sv(0) / sv(svsz-1); // condition number
    return mypinv(toSVD + svd.matrixV().transpose() * sv.asDiagonal() * svd.matrixV()) * XT * A * A;
}



/*! Function that prints a std::vector of doubles, used for debug.
    \param v A std::vector of doubles.
*/
void printVectord(vector<double> v, char * name) {
    clog << name << ":" << endl;
    for (int i = 0; i < v.size(); i++) {
        clog << v[i] << ",";
    }
    clog << endl;
}

/*! Function that prints an array of doubles, used for debug.
    \param v A std::vector of doubles.
*/
void printArrayd(double v[], int size, char name[]) {
    clog << name << ":" << endl;
    for (int i = 0; i < size; i++) {
        clog << v[i] << ",";
    }
    clog << endl;
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
