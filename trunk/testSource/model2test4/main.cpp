// model2test4.cpp : Defines the entry point for the console application.
//
#include "OMD.h"
#include "Model2.h"
#include <iostream>

using namespace std;
using namespace OMD;

void justOneRevJoint()
{
    Mat3x3 crap;
	crap << 1,0,0,
			0,1,0,
			0,0,1;

	Mat3x3 eye;
	eye << 1,0,0,
			0,1,0,
			0,0,1;

	Mat3x3 m2;
	m2 <<	0,0,1,
			1,0,0,
			0,1,0;

	Mat3x3 zero;
	zero << 0,0,0,
			0,0,0,
			0,0,0;

	double g = 9.80665;
	Model2 mymodel;

	Mat3x3 sliderI;
	sliderI << 0.065, 0, 0,
		          0, 0.065, 0,
				  0, 0, 0.026;

	BodyRigid *slider = mymodel.addBodyRigid("slider",15.6,sliderI,eye,false);
	BodyRigid *bob = mymodel.addBodyRigid("bob",12.0,eye*2.3E-002,eye,false);

	JointTranslational *j1 = mymodel.addJointTranslational("j1","DefaultIRF",Vect3(0,0,0.1),"slider",Vect3(0,0,0),Vect3(1,0,-2).normalized(),0,0);

	JointRevolute *j2 = mymodel.addJointRevolute("j2",slider,Vect3(0.1,0,-0.1),bob,Vect3(0.15,0,0),Vect3(0,0,1),0,0);

	mymodel.buildTree();

	Vect3 frc(0,-g*15.6,0);
	Force1Body *ballgrav = mymodel.addForce1Body("ballgrav",slider,frc,Vect3(0,0,0),false);
	frc =Vect3(0,-g*12.0, 0);
	Force1Body *ball2grav = mymodel.addForce1Body("ball2grav",bob,frc,Vect3(0,0,0),false);

	double time = 0.0;
	double dt = 0.001;
	double endtime = 1.0;

	Vect3 pos;
	Vect3 accel, accel_ang;
	while (time < endtime)
	{

		pos = bob->getPosition(Vect3(0,0,0));
		//std::cout<< time << ", " << pos.x() << " , "<< pos.y() << " , "<< pos.z() ;

		mymodel.integrate(time,time+dt,true);

		accel = bob->m_accel;
		accel_ang = bob->getAngularAccelerationGlobal();
		//std::cout<< " , " << accel.x() << " , "<< accel.y() << " , "<< accel.z() ;
		//std::cout<< " , " << accel_ang.x() << " , " << accel_ang.y() << " , " << accel_ang.z() << std::endl;
		time = time + dt;

		int crap = 1;

	}

		// This is the test to see if something has changed
		// Known answers
		Vect3 knownp(0.249369, -0.0143807, -0.000120165);
		Vect3 knowna(-2.36509, -8.80735, -0.450493);
		Vect3 knownaa(0,0,-60.1008);

		Vect3 errorp= knownp-pos;
		Vect3 errora= knowna-accel;
		Vect3 erroraa= knownaa-accel_ang;
		//std::cout << "Error: " << errorp.x() << ", " << errorp.y() << ", " << errorp.z() << std::endl;
		//std::cout << "Error: " << errora.x() << ", " << errora.y() << ", " << errora.z() << std::endl;
		//std::cout << "Error: " << erroraa.x() << ", " << erroraa.y() << ", " << erroraa.z() << std::endl;
		
		if ((fabs(errorp.x()) > 0.0000005) | (fabs(errorp.y()) > 0.00000005) | (fabs(errorp.z()) > 0.0000000005) | (fabs(errora.x()) > 0.000005) | (fabs(errora.y()) > 0.000005) | (fabs(errora.z()) > 0.0000005) | (fabs(erroraa.x()) > 0.000005) | (fabs(erroraa.y()) > 0.000005) | (fabs(erroraa.z()) > 0.00005)) 
		{
			std::cout << "Fail" << std::endl;
		}
		else
		{
			std::cout << "Pass" << std::endl;
		}
		// End of Test

}



//int _tmain(int argc, _TCHAR* argv[])
int main()
{

	justOneRevJoint();

	return 0;

}
