#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

int main (int argc, char** argv)
{
	Quaterniond q1(0.55, 0.3, 0.2, 0.2), q2(-0.1, 0.3, -0.7, 0.2);
	q1.normalize();
	q2.normalize();
	Vector3d t1(0.7, 1.1, 0.2), t2(-0.1, 0.4, 0.8);
	Vector3d p1(0.5, -0.1, 0.2);
	
	Isometry3d T1w(q1), T2w(q2);
	T1w.pretranslate(t1);	
	T2w.pretranslate(t2);
	
	Vector3d p2 = T2w * T1w.inverse() * p1;
	std::cout << p2.transpose() << std::endl;
	
	return 0;
}
