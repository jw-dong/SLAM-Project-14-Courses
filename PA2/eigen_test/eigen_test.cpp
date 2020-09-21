#include<iostream>
#include<Eigen/Core>
#include<Eigen/Dense>

using namespace Eigen;

int main(int argc, char** argv)
{
	
	Matrix<double, Dynamic, Dynamic> matrix_NN = MatrixXd::Random(100, 100);
	matrix_NN = matrix_NN * matrix_NN.transpose();  // 保证半正定
  	Matrix<double, Dynamic, 1> v_Nd = MatrixXd::Random(100, 1);
  	Matrix<double, Dynamic, 1> x;
  	
  	// QR分解
  	x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
  	std::cout << "(QR) x = " << x.transpose() << std::endl;

  	// cholesky分解
  	x = matrix_NN.ldlt().solve(v_Nd);
  	std::cout << "(cholesky) x = " << x.transpose() << std::endl;

	return 0;
}


