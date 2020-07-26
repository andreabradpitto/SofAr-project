#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "Eigen/SVD"

using namespace std;
using namespace Eigen;

int main() {
	double myconstant = 2;
	vector<double> V1 {1,1,1,1};
	vector<double> V2 {1,1,1,1};
	transform(V2.begin(), V2.end(), V2.begin(), [&myconstant](double& c){return c*myconstant;});
	transform (V1.begin(), V1.end(), V2.begin(), V1.begin(), std::plus<double>());
	for (int i = 0; i < 4; i++) {
		cout << V1[0] << ",";
	}
	return 0;
	
	/*MatrixXd m = MatrixXd::Random(3,2);
	cout << "Here is the matrix m:" << endl << m << endl;
	cout << "m has size:" << endl << m.rows() << endl;
	cout << "m.data:" << endl << m.data() << endl;
	m(0,0) = 4;
	cout << "Here is the new matrix m:" << endl << m << endl;
	JacobiSVD<MatrixXd> svd(m, ComputeThinU | ComputeThinV);
	cout << "Its singular values are:" << endl << svd.singularValues() << endl;
	cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
	cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
	VectorXd v(7);
	v(0) = 545;
	cout << "Here is the vector v:" << endl << v << endl;
	cout << "v has size:" << endl << v.rows() << endl;


	cout << "Here is the matrix m*m:" << endl << m*m.transpose() << endl;

    MatrixXd W = v.asDiagonal();

    cout << W << endl;

	vector<double> vec(v.data(), v.data() + v.size());

	for (int i = 0; i < 7; i++) {
		cout << vec[i] << endl;
	}

	vector<double> vec2 {5, 6, 7.0, 80, 90};
	vector<double> vec3 {5, 80, 90};
	MatrixXd vec2tom = Map<MatrixXd>(vec2.data(),2,2).transpose();
	VectorXd vec2tail4 = Map<VectorXd>(vec2.data()+1,4);
	cout << vec2tom << endl;
	cout << vec2tail4 << endl;

	VectorXd v1 = Map<VectorXd>(vec2.data(),3);
	VectorXd v2 = Map<VectorXd>(vec3.data(),3);
	cout << "joined v1 v2\n";
	VectorXd v12 (v1.size() + v2.size());
	v12 << v1, v2;
	cout << v12 << endl;
	cout << endl << endl;

	VectorXd ones5 = VectorXd::Constant(10,1);
	cout << "Ten ones:" << endl << ones5 << endl;*/

}