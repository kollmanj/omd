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
	zero <<		0,0,0,
				0,0,0,
				0,0,0;

	double g = -9.81;
	double bobmass = 2;
	double slidermass = 3;
	Vect3 jntAxis(0,1,0);

    Mat3x3 bobI = eye;
	Mat3x3 sliderI;
	sliderI <<	3.0,0.0,0.0,
		        0.0,3.0,0.0,
				0.0,0.0,3.0;

	Model3 mymodel3;

	BodyRigid *irf3 = mymodel3.addBodyRigid("irf3",1.0,eye,Vect3(0,0,0),eye,Vect3(0,0,0),Vect3(0,0,0),true);

	BodyRigid *bob3 = mymodel3.addBodyRigid("bob3",bobmass,bobI,Vect3(1,0,0),eye,Vect3(0,0,0),Vect3(0,0,0));
    BodyRigid *slider3 = mymodel3.addBodyRigid("slider3",slidermass,sliderI,Vect3(1,0,0),eye,Vect3(0,0,0),Vect3(0,0,0));

	JointSpherical *js3 = mymodel3.addJointSpherical("js3",irf3,Vect3(0,0,0),bob3);
	JointTranslational *jt3 = mymodel3.addJointTrans("jt3",bob3,Vect3(0,0,0),slider3,Vect3(1,0,0));
	Force1Body *bf3 = mymodel3.addForce1Body("bf3",bob3,Vect3(0,g*bobmass,g*bobmass),Vect3(0,0,0),false);

	//////////Model1 /////////////////
	Model2 mymodel1;

	BodyRigid *irf1 = mymodel1.addBodyRigid("irf1",0,zero,eye,true);
	// itermediate bob to get spherical jnt
	BodyRigid *bob1ia = mymodel1.addBodyRigid("bob1ia",0.0,zero,eye,false);
	BodyRigid *bob1ib = mymodel1.addBodyRigid("bob1ib",0.0,zero,eye,false);
	BodyRigid *bob1 = mymodel1.addBodyRigid("bob1",bobmass,bobI,eye,false);
	BodyRigid *slider1 = mymodel1.addBodyRigid("slider1",slidermass,sliderI,eye,false);

	JointRevolute *jrev1a = mymodel1.addJointRevolute("jrev1",irf1,Vect3(0,0,0),bob1ia,Vect3(0,0,0),Vect3(1,0,0));
	JointRevolute *jrev1b = mymodel1.addJointRevolute("jrev1b",bob1ia,Vect3(0,0,0),bob1ib,Vect3(0,0,0),Vect3(0,1,0));
	JointRevolute *jrev1 = mymodel1.addJointRevolute("jrev1",bob1ib,Vect3(0,0,0),bob1,Vect3(1,0,0),Vect3(0,0,1),0,0);
	JointTranslational *jtrans1 = mymodel1.addJointTranslational("jtrans1",bob1,Vect3(0,0,0),slider1,Vect3(0,0,0),Vect3(1,0,0),0,0);

	mymodel1.buildTree();

	Force1Body *bob1f = mymodel1.addForce1Body("bob1f",bob1,Vect3(0,g*bobmass,g*bobmass),Vect3(0,0,0),false);
	//////////Model1 /////////////////

	double time = 0.0;
	double dt = 0.001;
	double endtime = 2;

	Vect3 p23, p21, ps3, ps1, v23, v21;
	while (time < endtime)
	{
		mymodel3.calcIndependentStates();

        //std::cout << "time: " << time << std::endl;

		p23 = bob3->m_pos;
		//std::cout << "Model3 body2 pos: " << p23.x() << " " << p23.y() << " " << p23.z() << std::endl;
		p21 = bob1->m_pos;
		//std::cout << "Model1 body2 pos: " << p21.x() << " " << p21.y() << " " << p21.z() << std::endl;
		ps3 = slider3->m_pos;
		//std::cout << "Model3 slider pos: " << ps3.x() << " " << ps3.y() << " " << ps3.z() << std::endl;
		ps1 = slider3->m_pos;
		//std::cout << "Model1 slider pos: " << ps1.x() << " " << ps1.y() << " " << ps1.z() << std::endl;

		v23 = bob3->getVelocityGlobal();
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
	Vect3 knownpb2(-0.389404, -0.651285, -0.6513);
	Vect3 knownpb2b(-0.389408, -0.651291, -0.651291);
	Vect3 knownpslider(-2.00615, -3.35532, -3.3554);
	Vect3 knownpsliderb(-2.00615, -3.35532, -3.3554);
	Vect3 knownvb2(-0.202882, 0.0606924, 0.0606098);
	Vect3 knownvb2b(-0.202886, 0.0606529, 0.0606529);

	Vect3 errorpb2= knownpb2-p23;
	Vect3 errorpb2b = knownpb2b-p21;
	Vect3 errorpslider = knownpslider-ps3;
	Vect3 errorpsliderb = knownpsliderb-ps1;

	Vect3 errorvb2 = knownvb2-v23;
	Vect3 errorvb2b = knownvb2b-v21;

	//std::cout << "Error: " << errorpb2.x() << ", " << errorpb2.y() << ", " << errorpb2.z() << std::endl;
	//std::cout << "Error: " << errorpb2b.x() << ", " << errorpb2b.y() << ", " << errorpb2b.z() << std::endl;

	//std::cout << "Error: " << errorpslider.x() << ", " << errorpslider.y() << ", " << errorpslider.z() << std::endl;
	//std::cout << "Error: " << errorpsliderb.x() << ", " << errorpsliderb.y() << ", " << errorpsliderb.z() << std::endl;

	//std::cout << "Error: " << errorvb2.x() << ", " << errorvb2.y() << ", " << errorvb2.z() << std::endl;
	//std::cout << "Error: " << errorvb2b.x() << ", " << errorvb2b.y() << ", " << errorvb2b.z() << std::endl;

	if ((fabs(errorpb2.x()) > 0.0000005) | (fabs(errorpb2.y()) > 0.0000005) | (fabs(errorpb2.z()) > 0.00005) | 
		(fabs(errorpb2b.x()) >0.0000005) | (fabs(errorpb2b.y()) >0.0000005) | (fabs(errorpb2b.z()) >0.0000005) | 
		(fabs(errorpslider.x()) > 0.000005) | (fabs(errorpslider.y()) > 0.000005) | (fabs(errorpslider.z()) > 0.00005) |
		(fabs(errorpsliderb.x()) > 0.000005) | (fabs(errorpsliderb.y()) > 0.000005) | (fabs(errorpsliderb.z()) > 0.00005) |
		(fabs(errorvb2.x()) > 0.0000005) | (fabs(errorvb2.y()) > 0.00000005) | (fabs(errorvb2.z()) > 0.00000005) |
		(fabs(errorvb2b.x()) > 0.0000005) | (fabs(errorvb2b.y()) > 0.00000005) | (fabs(errorvb2b.z()) > 0.00000005)  )
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
