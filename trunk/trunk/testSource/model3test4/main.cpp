#include "Model3.h"
#include "Model2.h"
#include <iostream>

using namespace std;
using namespace OMD;

// model of a mass running down an incline plane on a translational joint

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
	body2I <<	3.0,0.0,0.0,
				0.0,3.0,0.0,
				0.0,0.0,3.0;

	Model3 mymodel3;

	Vect3 initialBody1Location(1,2,3);
	Vect3 transVelB1(0,0,0);
	Vect3 rotVelB1(0,0,0);

	Vect3 initialBody2Location(2,2,3);
	Vect3 transVelB2(0,0,0);
	Vect3 rotVelB2(0,0,0);
	Vect3 b2force(0,0,g*body2mass);
	Vect3 b2torque(0,0,0);
	Vect3 jntAxis(-1,0,-1);

	BodyRigid *irf3 = mymodel3.addBodyRigid("irf3",1.0,eye,initialBody1Location,eye,Vect3(0,0,0),Vect3(0,0,0),true);

	BodyRigid *body2 = mymodel3.addBodyRigid("body2",body2mass,body2I,initialBody2Location,eye,transVelB2,rotVelB2);

	JointTranslational *trans1 = mymodel3.addJointTrans("trans1",irf3,Vect3(1,0,0),body2,jntAxis);
    Force1Body *b2f3 = mymodel3.addForce1Body("b2f3",body2,b2force,b2torque);

	//////////Model1 /////////////////
	Model2 mymodel1;

	BodyRigid *irf1 = mymodel1.addBodyRigid("irf1",0,zero,eye,true);

	BodyRigid *bob1 = mymodel1.addBodyRigid("bob1",body2mass,body2I,eye,false);

	JointTranslational *trans11 = mymodel1.addJointTranslational("trans11",irf1,Vect3(1,2,3),bob1,Vect3(1,0,0),jntAxis,0,0);

	mymodel1.buildTree();

	Force1Body *bob1f = mymodel1.addForce1Body("bob1f",bob1,b2force);

	//////////Model1 /////////////////


	double time = 0.0;
	double dt = 0.0005;
	double endtime = 4;

	Vect3 p23, p21, v23, v21;
	while (time < endtime)
	{
		mymodel3.calcIndependentStates();

        //std::cout << "time: " << time << std::endl;

		p23 = body2->m_pos;
		//std::cout << "Model3 body2 pos: " << p23.x() << " " << p23.y() << " " << p23.z() << std::endl;
		p21 = bob1->m_pos;
		//std::cout << "Model1 body2 pos: " << p21.x() << " " << p21.y() << " " << p21.z() << std::endl;

		v23 = body2->getVelocityGlobal();
		//std::cout << "Model3 body2 vel: " << v23.x() << " " << v23.y() << " " << v23.z() << std::endl;
		v21 = bob1->getVelocityGlobal();
		//std::cout << "Model1 body2 vel: " << v21.x() << " " << v21.y() << " " << v21.z() << std::endl;

        if ( time > 0.1151 )
        {
            int crap =1;
        }

        mymodel3.integrate(time,time+dt,true);
		mymodel1.integrate(time,time+dt,true);

		time = time + dt;
	}

	// This is the test to see if something has changed //////////////////////////////////////////////////////////////////
	// Known answers
	Vect3 knownpb2(-37.2302, 2, -36.2302);
	Vect3 knownvb2(-19.6175, -0, -19.6175);

	Vect3 errorpb2= knownpb2-p23;
	Vect3 errorpb2b = knownpb2-p21;

	Vect3 errorvb2 = knownvb2-v23;
	Vect3 errorvb2b = knownvb2-v21;


	//std::cout << "Error: " << errorpb2.x() << ", " << errorpb2.y() << ", " << errorpb2.z() << std::endl;
	//std::cout << "Error: " << errorpb2b.x() << ", " << errorpb2b.y() << ", " << errorpb2b.z() << std::endl;

	//std::cout << "Error: " << errorvb2.x() << ", " << errorvb2.y() << ", " << errorvb2.z() << std::endl;
	//std::cout << "Error: " << errorvb2b.x() << ", " << errorvb2b.y() << ", " << errorvb2b.z() << std::endl;

	if ((fabs(errorpb2.x()) > 0.00005) | (fabs(errorpb2.y()) > 0.000005) | (fabs(errorpb2.z()) > 0.00005) | 
		(fabs(errorpb2b.x()) > 0.00005) | (fabs(errorpb2b.y()) > 0.000005) | (fabs(errorpb2b.z()) > 0.00005) | 
		(fabs(errorvb2.x()) > 0.00005) | (fabs(errorvb2.y()) > 0.000005) | (fabs(errorvb2.z()) > 0.00005) |
		(fabs(errorvb2b.x()) > 0.00005) | (fabs(errorvb2b.y()) > 0.000005) | (fabs(errorvb2b.z()) > 0.00005)  )
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
