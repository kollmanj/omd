// model1test1.cpp : Defines the entry point for the console application.
//

#include "OMD.h"
#include "Model1.h"
#include <iostream>

using namespace std;
using namespace OMD;

void ball1()
{
	std::vector<double> pos;
	pos.push_back(0);
	pos.push_back(0);
	pos.push_back(0);
	std::vector<double> eye_;
	eye_.push_back(1);	eye_.push_back(0);	eye_.push_back(0);
	eye_.push_back(0);	eye_.push_back(1);	eye_.push_back(0);
	eye_.push_back(0);	eye_.push_back(0);	eye_.push_back(1);
	std::vector<double> q_;
	q_.push_back(1);
	q_.push_back(0);
	q_.push_back(0);
	q_.push_back(0);
	std::vector<double> z_;
	z_.push_back(0);
	z_.push_back(0);
	z_.push_back(0);
	std::vector<double> initialLocation_;
	initialLocation_.push_back(0);
	initialLocation_.push_back(0);
	initialLocation_.push_back(1.5);

	Mat3x3 eye;
	eye << 1,0,0,
		   0,1,0,
		   0,0,1;

	double g = 9.81;
	double ballmass = 2;
	Model1 mymodel;

	Vect3 initialLocation(0,0,1.5);
	Vect3 transVel(0,0,0);
	Vect3 rotVel(0,0,0);
	//BodyRigid *ball = mymodel.addBodyRigid("ball",ballmass,OMD::eye(),initialLocation,Quat(1,0,0,0),transVel,rotVel,false);
	BodyRigid *ball = mymodel.addBodyRigid("ball",ballmass,eye_,initialLocation_,q_,z_,z_,false);

	Vect3 frc(0,0,-g*ballmass);
	Force1Body *ballgrav = mymodel.addForce1Body("ballgrav",ball,frc,Vect3(0,0,0),false);

	double stiff = 20000;
	double damp = 10;
	double frict = 0.6;
	double thresh = 0.001;
	ForceContact *fc = mymodel.addForceContact("fc",stiff,damp,frict,thresh);
	
	fc->addBox(1,1,0.1,pos,eye_);
	fc->addCapsule(1,0.0,pos,eye_,ball);

	//std::cout << ball->m_q.w()<< ", "<< ball->m_q.x()<< ", "<< ball->m_q.y()<< ", "<< ball->m_q.z() << std::endl;

	//ball->m_q.y() = 1.0;

	//std::cout << ball->m_q.w()<< ", "<< ball->m_q.x()<< ", "<< ball->m_q.y()<< ", "<< ball->m_q.z() << std::endl;

	double time = 0.0;
	double dt = 0.0005;
	double endtime = 1;

	while (time < endtime)
	{
		Vect3 pos = ball->getPosition(Vect3(0,0,0));
		std::cout<< time << " , " << pos.x() << " , "<< pos.y() << " , "<< pos.z() << std::endl;

		mymodel.integrate(time,time+dt);
		time = time + dt;
	}
	int crap = 1;
}




//int _tmain(int argc, _TCHAR* argv[])
int main()
{

	ball1();
	return 0;
}

