#include "Model2.h"
#include <iostream>

using namespace std;
using namespace OMD;

void pendumulum3()
{
	Mat3x3 eye;
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
	double ballmass = 1.0;
	Model2 mymodel;

	BodyRigid *irf = mymodel.addBodyRigid("irf",0,zero,eye,true);

	BodyRigid *bulb = mymodel.addBodyRigid("bulb",ballmass/5,eye,m2);

	BodyRigid *bulbi1 = mymodel.addBodyRigid("bulbi1",0,zero,eye);
	BodyRigid *bulbi2 = mymodel.addBodyRigid("bulbi2",0,zero,eye);

	Vect3 jntAxis(1,0,0);

	Vect3 p2j(0.0,2,0.0);
	Vect3 j2c(0,0,0);
	mymodel.addJointRevolute("pendJnt",irf,p2j,bulbi1,j2c,jntAxis,0.0,3.0);

	p2j = Vect3(0,0,0);
    jntAxis=Vect3(0,1,0);
    mymodel.addJointRevolute("pendJnt",bulbi1,p2j,bulbi2,j2c,jntAxis,0.0,5.0);
    jntAxis=Vect3(0,0,1);
    j2c = Vect3(0,-1.0,0);
    mymodel.addJointRevolute("pendJnt",bulbi2,p2j,bulb,j2c,jntAxis,0,7);

mymodel.buildTree();

Vect3 frc(0,-g*ballmass,0);
Force1Body *bulbgrav = mymodel.addForce1Body("bulbgrav",bulb,frc,Vect3(0,0,0),false);

	double time = 0.0;
	double dt = 0.0005;
	double endtime = 1.0;

	Vect3 pos;
	while (time < endtime)
	{
		mymodel.integrate(time,time+dt);
		time = time + dt;

        pos = bulb->m_pos;
		//std::cout<< time << " , " << pos.x() << " , "<< pos.y() << " , "<< pos.z() << std::endl;
	}

		// This is the test to see if something has changed
		// Known answers
		Vect3 known(-0.382509, 1.1257, -0.298811);
		Vect3 error= known-pos;
		//std::cout << "Error: " << error.x() << ", " << error.y() << ", " << error.z() << std::endl;
		
		if ((fabs(error.x()) > 0.0000005) | (fabs(error.y()) > 0.00005) | (fabs(error.z()) > 0.0000005) ) 
		{
			std::cout << "Fail" << std::endl;
		}
		else
		{
			std::cout << "Pass" << std::endl;
		}
		// End of Test
}

int main()
{
	pendumulum3();
	return 0;
}
