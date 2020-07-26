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

#define ETA 0
#define JOINTS_MAJ 3
#define JOINTS_MARGIN 0.2
#define JOINTS_MAXPOLE 1
#define Kp 1
#define Kv 25
#define Krot 25
#define NJOINTS 7
#define NUM_IK_SERVICES 3
#define NUM_IK_SOLUTIONS 4 // will be 6
#define NUM_OPTIMIZED_SOLUTIONS 4
#define REG_THR 0.1
#define SPACE_DOFS 6
#define VEL_MAJ 3
#define VEL_MARGIN 0.2
#define VEL_MAXPOLE 20

const double QMIN[] = {-5555, 6, 7, 8, 3, 4, 5};
const double QMAX[] = {5000, 6, 7, 8, 3, 4, 5};
const double QDOTMIN[] = {5, 6, 7, 8, 3, 4, 5};
const double QDOTMAX[] = {5, 6, 7, 8, 3, 4, 5};
const double QINIT[] = {5, 6, 7, 8, 3, 4, 5};
const double WEIGHTS[] = {2, 2, 2, 2, 0, 0};
const double b = -log(0.5)/0.01;
const double DT = 0.01;
const MatrixXd ID_MATRIX_NJ = MatrixXd::Identity(NJOINTS,NJOINTS);
const MatrixXd ID_MATRIX_SPACE_DOFS = MatrixXd::Identity(SPACE_DOFS,SPACE_DOFS);
const VectorXd ONES_VEC_NJ = VectorXd::Constant(NJOINTS,1);
const VectorXd QDOT_FAV((VectorXd(NJOINTS) << 1, 2, 3, 4, 5, 6, 7).finished());
const MatrixXd ZERO_MATRIX_NJ = MatrixXd::Zero(NJOINTS,NJOINTS);

double bel;
 
class Safety {
public:
	double rdot;
	double Adiag;
    Safety(double rd,double Ad);
};

Safety::Safety(double rd, double Ad) {
    rdot=rd;Adiag=Ad;
}

class Ik {
public:
    VectorXd qdot1;
    VectorXd qdot2;
    Ik(VectorXd qd1,VectorXd qd2);
};


Ik::Ik(VectorXd qd1,VectorXd qd2) {
    qdot1=qd1;qdot2=qd2;
}


MatrixXd regPinv(MatrixXd X,MatrixXd A,MatrixXd Q,double eta) {
	// Adiag has size NJOINTS
    MatrixXd XT = X.transpose();
    MatrixXd idMinusQ = ID_MATRIX_NJ - Q;
    MatrixXd toSVD = XT*A*X + eta*(idMinusQ*idMinusQ.transpose());

    JacobiSVD<MatrixXd> svd(toSVD, ComputeThinU | ComputeThinV);
    VectorXd sv = svd.singularValues();

    for (int i = 0; i < NJOINTS; i++) {
  		bel = exp(-b*sv(i)*sv(i));
    	sv(i) = sv(i)/(sv(i)*sv(i) + bel*bel);
    }

    return svd.matrixV().transpose() * (sv.asDiagonal()) * (svd.matrixU().transpose()) * XT * A * A;
}



Ik computeqdot(VectorXd partialqdot,MatrixXd Q2,MatrixXd J,MatrixXd JL,
    MatrixXd JLdot,VectorXd qdot,VectorXd eta,VectorXd rho,VectorXd etadot,
    VectorXd rhodot,VectorXd v,VectorXd w,VectorXd a) {
    VectorXd ve1 = v + Kp*eta;
    VectorXd ve2 = DT*(a - JLdot*qdot + Kv*etadot + Kp*eta) + JL*qdot;
    VectorXd xedot1 = VectorXd(6);
    VectorXd xedot2 = VectorXd(6);
    xedot1 << Krot*rho, ve1;
    xedot2 << Krot*rho, ve2;
    MatrixXd JTimesQ2 = J*Q2;
    MatrixXd pinvAux = regPinv(JTimesQ2,ID_MATRIX_SPACE_DOFS,Q2,ETA);
    MatrixXd pinvQZero = regPinv(JTimesQ2,ID_MATRIX_SPACE_DOFS,ZERO_MATRIX_NJ,ETA);
    MatrixXd W2 = JTimesQ2*pinvAux;
    MatrixXd tempProduct1 = Q2*pinvQZero*W2;
    MatrixXd tempProduct2 = J*partialqdot;
    return Ik(partialqdot + tempProduct1 * (xedot1 - tempProduct2),
              partialqdot + tempProduct1 * (xedot2 - tempProduct2));
}



void computeCostResponse(MatrixXd J,VectorXd qdot1,VectorXd qdot2,math_pkg::Cost::Response &res) {
    VectorXd qdot1opt1,qdot1opt2,qdot2opt1,qdot2opt2;
    MatrixXd G = ID_MATRIX_NJ - (regPinv(J,ID_MATRIX_SPACE_DOFS,ID_MATRIX_NJ,ETA)) * J;
    MatrixXd GTimesGSharp = G*regPinv(G,ID_MATRIX_NJ,ID_MATRIX_NJ,ETA);
    VectorXd z1 = G*ONES_VEC_NJ;
    qdot1opt1 = qdot1 + z1;
    qdot1opt2 = qdot1 + GTimesGSharp*(QDOT_FAV - qdot1);
    qdot2opt1 = qdot2 + z1;
    qdot1opt2 = qdot1 + GTimesGSharp*(QDOT_FAV - qdot2);
    res.qdot1opt1.velocity = vector<double> (qdot1opt1.data(),qdot1opt1.data()+qdot1opt1.size());
    res.qdot1opt2.velocity = vector<double> (qdot1opt2.data(),qdot1opt2.data()+qdot1opt2.size());
    res.qdot2opt1.velocity = vector<double> (qdot2opt1.data(),qdot2opt1.data()+qdot2opt1.size());
    res.qdot2opt2.velocity = vector<double> (qdot2opt2.data(),qdot2opt2.data()+qdot2opt2.size());
}