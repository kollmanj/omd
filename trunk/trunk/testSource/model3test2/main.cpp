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
	zero <<	0,0,0,
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

	Vect3 initialBody1Location(1,2,3);
	Vect3 transVelB1(0,0,0);
	Vect3 rotVelB1(0,0,0);

	Vect3 initialBody2Location(2,2,3);
	Vect3 transVelB2(0,0,0);
	Vect3 rotVelB2(0,0,0);
	Vect3 b2force(0,0,g*body2mass);
	Vect3 b2torque(0,0,0);
	Vect3 jntAxis(1,0,0);

	BodyRigid *body1 = mymodel3.addBodyRigid("body1",body1mass,eye,initialBody1Location,eye,transVelB1,rotVelB1);
	BodyRigid *body2 = mymodel3.addBodyRigid("body2",body2mass,body2I,initialBody2Location,eye,transVelB2,rotVelB2);

	mymodel3.addJointRevolute("revjnt2",body1,Vect3(0,0,0.0),body2,jntAxis);
	Force1Body *b2f3 = mymodel3.addForce1Body("b2f3",body2,b2force,b2torque);
	Force1Body *b1f3 = mymodel3.addForce1Body("b1f3",body1,Vect3(0,0,-9.81*body1mass),Vect3(0,0,0));

	//////////Model1 /////////////////
	Model2 mymodel2;

	// make 6 dof body
	BodyRigid *irf1 = mymodel2.addBodyRigid("irf1",0,zero,eye,true);
	BodyRigid *b1 = mymodel2.addBodyRigid("i1",0,zero,eye,false);
	BodyRigid *b2 = mymodel2.addBodyRigid("i2",0,zero,eye,false);
	BodyRigid *b3 = mymodel2.addBodyRigid("i3",0,zero,eye,false);
	BodyRigid *b4 = mymodel2.addBodyRigid("i4",0,zero,eye,false);
	BodyRigid *b5 = mymodel2.addBodyRigid("i5",0,zero,eye,false);
	BodyRigid *ball = mymodel2.addBodyRigid("ball",body1mass,eye,eye,false);
	JointTranslational *j1 = mymodel2.addJointTranslational("j1",irf1,initialBody1Location,b1,Vect3(0,0,0),Vect3(1,0,0),0,0);
	JointTranslational *j2 = mymodel2.addJointTranslational("j2",b1,Vect3(0,0,0),b2,Vect3(0,0,0),Vect3(0,1,0),0,0);
	JointTranslational *j3 = mymodel2.addJointTranslational("j3",b2,Vect3(0,0,0),b3,Vect3(0,0,0),Vect3(0,0,1),0,0);
	JointRevolute *j4 = mymodel2.addJointRevolute("j4",b3,Vect3(0,0,0),b4,Vect3(0,0,0),Vect3(1,0,0),0,0);
	JointRevolute *j5 = mymodel2.addJointRevolute("j5",b4,Vect3(0,0,0),b5,Vect3(0,0,0),Vect3(0,1,0),0,0);
	JointRevolute *j6 = mymodel2.addJointRevolute("j6",b5,Vect3(0,0,0),ball,Vect3(0,0,0),Vect3(0,0,1),0,0);

	BodyRigid *bob1 = mymodel2.addBodyRigid("bob1",body2mass,body2I,eye,false);

	JointRevolute *jrev = mymodel2.addJointRevolute("jrev",ball,Vect3(0,0,0),bob1,Vect3(1,0,0),jntAxis,0,0);

	mymodel2.buildTree();

	Vect3 frc(0,0,-g*body2mass);
	Force1Body *ballf = mymodel2.addForce1Body("ballf",ball,Vect3(0,0,-9.81*body1mass),Vect3(0,0,0),false);
	Force1Body *bob1f = mymodel2.addForce1Body("bob1f",bob1,Vect3(0,0,-9.81*body2mass),Vect3(0,0,0),false);
	//////////Model1 /////////////////
	double time = 0.0;
	double dt = 0.0005;
	double endtime = 1.0225;
	Vect3 p13, p11, p23, p21, v13, v11, v23, v21;
	while (time < endtime)
	{
		mymodel3.calcIndependentStates();

		//std::cout << "time: " << time << std::endl;
		p13 = body1->m_pos;
		//std::cout << "Model3 body1 pos: " << p13.x() << ", " << p13.y() << ", " << p13.z() << std::endl;
		p11 = ball->m_pos;
		//std::cout << "Model1 body1 pos: " << p11.x() << ", " << p11.y() << ", " << p11.z()  << std::endl;

		p23 = body2->m_pos;
		std::cout << "Model3 body2 pos: " << p23.x() << ", " << p23.y() << ", " << p23.z()   << std::endl;
		p21 = bob1->m_pos;
		//std::cout << "Model1 body2 pos: " << p21.x() << ", " << p21.y() << ", " << p21.z()    << std::endl;

		v13 = body1->getVelocityGlobal();
		//std::cout << "Model3 body1 vel: " << v13.x() << ", " << v13.y() << ", " << v13.z() << std::endl;
		v11 = ball->getVelocityGlobal();
		//std::cout << "Mdoel1 body1 vel: " << v11.x() << ", " << v11.y() << ", " << v11.z() << std::endl;

		v23 = body2->getVelocityGlobal();
		//std::cout << "Model3 body2 vel: " << v23.x() << ", " << v23.y() << ", " << v23.z() << std::endl;
		v21 = bob1->getVelocityGlobal();
		//std::cout << "Model1 body2 vel: " << v21.x() << ", " << v21.y() << ", " << v21.z() << std::endl;

		mymodel3.integrate(time,time+dt,true);
		mymodel2.integrate(time,time+dt,true);

		time = time + dt;
	}

	// This is the test to see if something has changed //////////////////////////////////////////////////////////////////
	// Known answers
	Vect3 knownpb1(1, 2, -2.12821);
	Vect3 knownpb2(2, 2, -2.12821);
	Vect3 knownvb1(0,0,-10.0307);
	Vect3 knownvb2(0,0,-10.0307);

	Vect3 errorpb1= knownpb1-p13;
	Vect3 errorpb1b = knownpb1-p11;
	Vect3 errorpb2 = knownpb2-p23;
	Vect3 errorpb2b = knownpb2-p21;

	Vect3 errorvb1= knownvb1-v13;
	Vect3 errorvb1b = knownvb1-v11;
	Vect3 errorvb2 = knownvb2-v23;
	Vect3 errorvb2b = knownvb2-v21;

	//std::cout << "Error: " << errorpb1.x() << ", " << errorpb1.y() << ", " << errorpb1.z() << std::endl;
	//std::cout << "Error: " << errorpb1b.x() << ", " << errorpb1b.y() << ", " << errorpb1b.z() << std::endl;
	//std::cout << "Error: " << errorpb2.x() << ", " << errorpb2.y() << ", " << errorpb2.z() << std::endl;
	//std::cout << "Error: " << errorpb2b.x() << ", " << errorpb2b.y() << ", " << errorpb2b.z() << std::endl;

	//std::cout << "Error: " << errorvb1.x() << ", " << errorvb1.y() << ", " << errorvb1.z() << std::endl;
	//std::cout << "Error: " << errorvb1b.x() << ", " << errorvb1b.y() << ", " << errorvb1b.z() << std::endl;
	//std::cout << "Error: " << errorvb2.x() << ", " << errorvb2.y() << ", " << errorvb2.z() << std::endl;
	//std::cout << "Error: " << errorvb2b.x() << ", " << errorvb2b.y() << ", " << errorvb2b.z() << std::endl;

	if ((fabs(errorpb1.x()) > 0.000005) | (fabs(errorpb1.y()) > 0.000005) | (fabs(errorpb1.z()) > 0.000005) | 
		(fabs(errorpb1b.x()) > 0.000005) | (fabs(errorpb1b.y()) > 0.000005) | (fabs(errorpb1b.z()) > 0.000005) | 
		(fabs(errorpb2.x()) > 0.000005) | (fabs(errorpb2.y()) > 0.000005) | (fabs(errorpb2.z()) > 0.000005) |
		(fabs(errorpb2b.x()) > 0.000005) | (fabs(errorpb2b.y()) > 0.000005) | (fabs(errorpb2b.z()) > 0.000005) |

		(fabs(errorvb1.x()) > 0.0000005) | (fabs(errorvb1.y()) > 0.0000005) | (fabs(errorvb1.z()) > 0.00005) |
		(fabs(errorvb1b.x()) > 0.0000005) | (fabs(errorvb1b.y()) > 0.0000005) | (fabs(errorvb1b.z()) > 0.00005) |
		(fabs(errorvb2.x()) > 0.0000005) | (fabs(errorvb2.y()) > 0.0000005) | (fabs(errorvb2.z()) > 0.00005) |
		(fabs(errorvb2b.x()) > 0.0000005) | (fabs(errorvb2b.y()) > 0.0000005) | (fabs(errorvb2b.z()) > 0.00005) )
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
