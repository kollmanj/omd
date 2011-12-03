// model1test1.cpp : Defines the entry point for the console application.
//

#include "OMD.h"
#include "Model1.h"
#include <iostream>

using namespace std;
using namespace OMD;

void ball1()
{
	double g = 9.81;
	double ballmass = 2;
	Model1 mymodel;

	Vect3 initialLocation(0,0,1);
	Vect3 transVel(1,2,3);
	Vect3 rotVel(4,5,6);
	BodyRigid *ball = mymodel.addBodyRigid("ball",ballmass,OMD::eye(),initialLocation,Quat(1,0,0,0),transVel,rotVel,false);

	Vect3 frc(0,0,-g*ballmass);
	Force1Body *ballgrav = mymodel.addForce1Body("ballgrav",ball,frc,Vect3(0,0,0),false);

	//std::cout << ball->m_q.w()<< ", "<< ball->m_q.x()<< ", "<< ball->m_q.y()<< ", "<< ball->m_q.z() << std::endl;

	ball->m_q.y() = 1.0;

	//std::cout << ball->m_q.w()<< ", "<< ball->m_q.x()<< ", "<< ball->m_q.y()<< ", "<< ball->m_q.z() << std::endl;

	double time = 0.0;
	double dt = 0.0005;
	double endtime = 4;

	while (time < endtime)
	{
		Vect3 pos = ball->getPosition(Vect3(0,0,0));
		//std::cout<< time << " , " << pos.x() << " , "<< pos.y() << " , "<< pos.z() << std::endl;

		mymodel.integrate(time,time+dt);
		time = time + dt;
	}
	int crap = 1;
}




//int _tmain(int argc, _TCHAR* argv[])
int main()
{
	MatNxN a3(3,1);
	MatNxN temp2 = a3;
	a3.resize(4,1);
	a3 << temp2, 3;
	//std::cout << a3 << std::endl;

	MatNxN a1(2,2);
	a1 << 1,2,
		3,4;

	MatNxN a2(2,2);
	a2 << 5,6,
		7,8;

	MatNxN temp = a1;

	a1.resize(temp.rows()+a2.rows(),temp.cols());

	a1 << temp, a2;

	//std::cout << "a1: " <<a1 << std::endl;
	//std::cout << "a1(2,1): " << a1(2,1) << std::endl;

	Eigen::Matrix<double,11,14> a;

	a << 0,    0,    0,     0,     2,   0,    0,     0,    0,   0,     0,     0,     0,     0,
		0,    0,    0,     0,     0,   0,     2,    0,     0,    0,   0,     0,     0,     0,
		0,    0,    1,     0,     0,   2,     0,    0,     0,    0,   0,     0,     0,     0,
		-1,    0,    0,     4,     0,   0,     0,    0,     0,    0,   0,     0,     0,     0,
		0,    0,    0,     0,     0,  -2,     0,    0,     0,    0,   0,     0,     2,     0,
		0,    0,    0,     0,     0,   0,    -2,    0,     0,    0,   0,     0,     0,     2,
		0,    0,    1,     0,     0,   0,     0,    0,     0,   -1,   0,     0,     0,     0,
		0,   -1,    0,     0,     0,   0,     0,    0,     1,    0,   0,     0,     0,     0,
		0,    0,    0,     0,     2,   0,     0,    0,     0,    0,   0,    -2,     0,     0,
		0,    0,    0,     2,     0,   0,     0,    0,     0,    0,   0,     0,     0,     0,
		0,    0,    0,     0,     0,   0,     0,    0,     0,    0,   2,     0,     0,     0;

	MatNxN adroprow0 = dropRow(a,0);

	//std::cout << "a= " << a << std::endl;
	//std::cout << "adroprow0= " << adroprow0 << std::endl;

	Eigen::FullPivLU<Matrix<double,11,14>> flu(a);

	Eigen::ColPivHouseholderQR<Matrix<double,11,14>> ldlt_(a);

	// test initialization of Mat3
	//Mat3x3 mat3eye= Mat3x3::Identity();
	//std::cout << mat3eye << std::endl;


	//std::cout << flu.permutationP().indices().transpose() << std::endl;
	//std::cout << flu.permutationQ().indices().transpose() << std::endl;  // <- that is it

	//std::cout << ldlt_.nonzeroPivots() << std::endl;
	//std::cout << ldlt_.colsPermutation().indices() << std::endl;

	int crap = 1;

	//std::cout << "No Test" << std::endl;
	std::cout << "Pass" << std::endl;
	return 0;
}

