#include "Model3.h"
#include "Model2.h"

#include <iostream>

using namespace std;
using namespace OMD;

// make a model1 and model3 that represent the same thing to compare results
void ball1()
{
	Mat3x3 eye;
	eye <<	1,0,0,
			0,1,0,
			0,0,1;

	Mat3x3 zero;
	zero << 0,0,0,
			0,0,0,
			0,0,0;

	double g = -9.81;
	double body1mass = 2;
	double body2mass = 3;
	Mat3x3 body2I;
	body2I <<	3,0.0,0,
				0.0,3,0,
				0.0,0,3;

	Model3 mymodel3;

	Vect3 initialBody1Location(2,2,3);
	Vect3 transVelB1(0,0,0);
	Vect3 rotVelB1(0,0,0);

	Vect3 initialBody2Location(3,2,3);
	Vect3 transVelB2(0,0,0);
	Vect3 rotVelB2(0,0,0);
	Vect3 b2torque(0,0,0);
	Vect3 jntAxis(0,1,0);

	BodyRigid *irf3 = mymodel3.addBodyRigid("irf3",1.0,eye,Vect3(0,0,0),eye,Vect3(0,0,0),Vect3(0,0,0),true);

	BodyRigid *body1 = mymodel3.addBodyRigid("body1",body1mass,eye,initialBody1Location,eye);
    BodyRigid *body2 = mymodel3.addBodyRigid("body2",body2mass,body2I,initialBody2Location,eye);

	mymodel3.addJointRevolute("revjnt2",irf3,Vect3(1,2,3),body1,jntAxis);

	//mymodel3.addJointUniversal("univ",body1,Vect3(0,0,0),Vect3(0,0,1),body2,Vect3(1,0,0),Vect3(0,1,0));
	mymodel3.addJointUniversal("univ",body1,Vect3(0,0,0),Vect3(0,0,1),body2,Vect3(0,1,0));
    Force1Body *b1f3 = mymodel3.addForce1Body("b1f3",body1,Vect3(0,g*body1mass,g*body1mass),Vect3(0,0,0),false);
    Force1Body *b2f3 = mymodel3.addForce1Body("b2f3",body2,Vect3(0,g*body2mass,g*body2mass),Vect3(0,0,0),false);
	

	double time = 0.0;
	double dt = 0.001;
	double endtime = 1.0;

	Vect3 p23;
	while (time < endtime)
	{
		mymodel3.calcIndependentStates();


		p23 = body2->m_pos;

		//std::cout <<time << " " << p23.x() << " " << p23.y() << " " << p23.z() << std::endl;
	
        if ( time > 0.1151 )
        {
            int crap =1;
        }

        mymodel3.integrate(time,time+dt,true);

		time = time + dt;

	}
	// This is the test to see if something has changed //////////////////////////////////////////////////////////////////
	// Known answers
	Vect3 knownpb2( -0.0506668, 1.35613, 1.60517);

	Vect3 errorpb2= knownpb2-p23;

	//std::cout << "Error: " << errorpb2.x() << ", " << errorpb2.y() << ", " << errorpb2.z() << std::endl;

	if ((fabs(errorpb2.x()) > 0.00000005) | (fabs(errorpb2.y()) > 0.000005) | (fabs(errorpb2.z()) > 0.000005) )
	{
		std::cout << "Fail" << std::endl;
	}
	else
	{
		std::cout << "Pass" << std::endl;
	}
	// End of Test  //////////////////////////////////////////////////////////////////
}

int main()
{
	ball1();
	return 0;
}
