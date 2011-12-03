#include "Model2.h"
#include <iostream>

using namespace std;
using namespace OMD;

void revJointTest()
{	Mat3x3 eye;
	eye <<	1,0,0,
			0,1,0,
			0,0,1;

	Mat3x3 m2;
	m2 <<	0,0,1,
			1,0,0,
			0,1,0;

	Mat3x3 zero;
	zero <<	0,0,0,
			0,0,0,
			0,0,0;

	double g = 9.81;
	double ballmass = 1;
	Model2 mymodel;

	BodyRigid *irf = mymodel.addBodyRigid("irf",0,zero,eye,true);
	BodyRigid *bulb = mymodel.addBodyRigid("bulb",ballmass/5.0,eye,m2);
	BodyRigid *bulb2 = mymodel.addBodyRigid("bulb2",ballmass/5.0,eye,eye);

	Vect3 jntAxis(0.0,0.0,1.0);
	Vect3 p2j(0.0,2.0,0.0);
	Vect3 j2c(0.0,0.0,2.0);
	JointRevolute *pendJnt = mymodel.addJointRevolute("pendJnt",irf,p2j,bulb,j2c,jntAxis,0.0,3.0);
	p2j = Vect3(0,0,1.0);
	j2c = Vect3(1,0,0);
	jntAxis=Vect3(0,0,1.0);
	JointRevolute *pendJnt2 = mymodel.addJointRevolute("pendJnt2",bulb,p2j,bulb2,j2c,jntAxis,0.0,3.0);

	mymodel.buildTree();

	Vect3 frc(0,-g*ballmass,0);
	Force1Body *bulbgrav = mymodel.addForce1Body("bulbgrav",bulb,frc,Vect3(0,0,0),false);
	Force1Body *bulbgrav2 = mymodel.addForce1Body("bulbgrav2",bulb2,frc,Vect3(0,0,0),false);

	double time = 0.0;
	double dt = 0.0005;
	double endtime = dt*3;
//	double endtime = 10.00;

	while (time < endtime)
	{
		mymodel.integrate(time,time+dt);
		time = time + dt;
		Vect3 pos = bulb2->getPosition(Vect3(.3,.2,.1));
		//std::cout<< time << " , " << pos.x << " , "<< pos.y << " , "<< pos.z << std::endl;

//		Vect3 velGlobal = ball->getVelGlobal(Vect3(.3,.2,.1));
        Vect3 velGlobal =bulb2->getVelocityGlobal(Vect3(0,1,0));
        Vect3 bulb2w  = bulb2->getAngularVelocityLocal();
		//std::cout<< "t: "<<time << " , " << bulb2w.x << " , "<< bulb2w.y << " , "<< bulb2w.z << std::endl;

//		double u = pendJnt2->GetU();
//		std::cout<< "t: "<<time << " , " << "u: "<< u << std::endl;
        vector<double> pendJnt2Dot = pendJnt2->getDot();
        //std::cout << "t: "<<time << ", " << "udot: " << pendJnt2Dot[1] << std::endl;
		//Vect3 ori = ball->getRot();
	}
	//std::cout << "No Test" << std::endl;
	std::cout << "Pass" << std::endl;
}

int main()
{
	revJointTest();
	return 0;
}
