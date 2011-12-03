// model2test2.cpp : Defines the entry point for the console application.
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

	double g = 9.81;
	double ballmass = 1;
	Model2 mymodel;

	// making this body causes a crash in addJointRevolute
	//BodyRigid *irf = mymodel.addBodyRigid("irf",1,eye,eye,false);

	OMD::Vect3 initialLocation(0,0,1);
	Vect3 transVel(1,2,3);
	Vect3 rotVel(4,5,6);

	BodyRigid *ball = mymodel.addBodyRigid("ball",ballmass,eye,eye,false);
	BodyRigid *ball2 = mymodel.addBodyRigid("ball2",ballmass,eye,eye,false);
	//Body *ball = mymodel.addBody("ball",ballmass,eye,m2,false);
	//JointRevolute *j1 = mymodel.addJointRevolute("j1",irf,Vect3(0,0,0),ball,Vect3(1,0,0),Vect3(0,1,0),0,0);
	//JointRevolute *j1 = mymodel.addJointRevolute("j1","irf",Vect3(0,0,0),"ball",Vect3(1,0,0),Vect3(0,1,0),0,0);
	JointRevolute *j1 = mymodel.addJointRevolute("j1","DefaultIRF",Vect3(0,0,0),"ball",Vect3(1,0,0),Vect3(0,1,0),0,0);
	JointRevolute *j2 = mymodel.addJointRevolute("j2","ball",Vect3(0,0,0),"ball2",Vect3(1.5,0,0),Vect3(0,1,0),0,0);

	mymodel.buildTree();

	Vect3 frc(0,0,-g*ballmass);
	Force1Body *ballgrav = mymodel.addForce1Body("ballgrav",ball,frc,Vect3(0,0,0),false);
	Force1Body *ball2grav = mymodel.addForce1Body("ball2grav",ball2,frc,Vect3(0,0,0),false);

	double time = 0.0;
	double dt = 0.001;
	double endtime = 2.0;

	Vect3 pos;
	Vect3 accel, accel_ang;
	while (time < endtime)
	{
		pos = ball2->getPosition(Vect3(0,0,0));
		//std::cout<< time << ", " << pos.x() << " , "<< pos.y() << " , "<< pos.z() ;

		mymodel.integrate(time,time+dt,true);

		accel = ball2->m_accel;
		accel_ang = ball2->m_wldot;
		//std::cout<< " , " << accel.x() << " , "<< accel.y() << " , "<< accel.z() ;
		//std::cout<< " , " << accel_ang.x() << " , " << accel_ang.y() << " , " << accel_ang.z() << std::endl;
		time = time + dt;

		int crap = 1;
	}

	
		// This is the test to see if something has changed
		// Known answers
		Vect3 knownp(-1.9221,0,0.446528);
		Vect3 knowna(0.451256,0,-7.41767);
		Vect3 knownaa(0,-3.09472,0);
		Vect3 errorp= knownp-pos;
		Vect3 errora= knowna-accel;
		Vect3 erroraa= knownaa-accel_ang;
		//std::cout << "Error: " << errorp.x() << ", " << errorp.y() << ", " << errorp.z() << std::endl;
		//std::cout << "Error: " << errora.x() << ", " << errora.y() << ", " << errora.z() << std::endl;
		//std::cout << "Error: " << erroraa.x() << ", " << erroraa.y() << ", " << erroraa.z() << std::endl;
		
		if ((fabs(errorp.x()) > 0.00005) | (fabs(errorp.y()) > 0.00005) | (fabs(errorp.z()) > 0.0000005) | (fabs(errora.x()) > 0.0000005) | (fabs(errora.y()) > 0.00005) | (fabs(errora.z()) > 0.000005) | (fabs(erroraa.x()) > 0.00001) | (fabs(erroraa.y()) > 0.000005) | (fabs(erroraa.z()) > 0.000005)) 
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
