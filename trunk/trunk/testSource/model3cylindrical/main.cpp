// model2test4.cpp : Defines the entry point for the console application.
//
#include "OMD.h"
#include "Model3.h"
#include <iostream>

using namespace std;
using namespace OMD;

void ball1()
{
	Mat3x3 eye;
	eye << 1,0,0,
		   0,1,0,
		   0,0,1;

	Mat3x3 zero;
	zero << 0,0,0,
			0,0,0,
			0,0,0;

	double g = -9.81;
	double bobmass = 2;
	double slidermass = 3;

	Vect3 jntAxis;
	jntAxis << 0,1,0;

	Mat3x3 bobI = eye;

	Mat3x3 sliderI;
	sliderI <<	3.0,0.0,0.0,
		        0.0,3.0,0.0,
				0.0,0.0,3.0;

	Model3 mymodel3;

	BodyRigid *irf3 = mymodel3.addBodyRigid("irf3",1.0,eye,Vect3(0,0,0),eye,Vect3(0,0,0),Vect3(0,0,0),true);
	BodyRigid *bob3 = mymodel3.addBodyRigid("bob3",bobmass,bobI,Vect3(1,0,0),eye,Vect3(0,0,0),Vect3(0,0,0));
    BodyRigid *slider3 = mymodel3.addBodyRigid("slider3",slidermass,sliderI,Vect3(1,0,0),eye,Vect3(0,0,0),Vect3(0,0,0));

	JointCylindrical *js3 = mymodel3.addJointCylindrical("js3",irf3,Vect3(0,0,0),bob3,jntAxis);
	//JointTranslational *jt3 = mymodel3.addJointTrans("jt3",bob3,Vect3(0,0,0),slider3,Vect3(0,0,0),Vect3(1,0,0));
	JointTranslational *jt3 = mymodel3.addJointTrans("jt3",bob3,Vect3(0,0,0),slider3,Vect3(1,0,0));
	
	Vect3 frc(0,g*bobmass,g*bobmass);
	Force1Body *bf3 = mymodel3.addForce1Body("bf3",bob3,frc,Vect3(0,0,0),false);

	double time = 0.0;
	double dt = 0.001;
	double endtime = 2.0;

	Vect3 slider3Pos;
	while (time < endtime)
	{
		mymodel3.calcIndependentStates();

		slider3Pos = slider3->getPosition(Vect3(0,0,0));
		//std::cout << "time: " << time << std::endl;
		//std::cout << "slider Position: " << slider3Pos.x() << ", " << slider3Pos.y() << ", " << slider3Pos.z() << std::endl;

		mymodel3.integrate(time,time+dt,true);
		time = time + dt;
	}

		// This is the test to see if something has changed
		// Known answers
		Vect3 knownp( -1.16121, -7.848, -3.73494);

		Vect3 errorp= knownp-slider3Pos;
		//std::cout << "Error: " << errorp.x() << ", " << errorp.y() << ", " << errorp.z() << std::endl;
		
		if ((fabs(errorp.x()) > 0.000005) | (fabs(errorp.y()) > 0.0005) | (fabs(errorp.z()) > 0.000005)) 
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

	ball1();

	return 0;

}
