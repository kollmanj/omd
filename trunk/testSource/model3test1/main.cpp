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

	double g = 9.81;
	double body1mass = 2;
	double body2mass = 3;
	
	Mat3x3 body2I;
	body2I <<	3.0,0.0,0.0,
				0.0,3.0,0.0,
				0.0,0.0,3.0;

	Model3 mymodel3;

	Vect3 initialBody1Location;
	initialBody1Location << 1,2,3;
	Vect3 transVelB1(0,0,0);
	Vect3 rotVelB1(0,0,0);

	Vect3 initialBody2Location(1,2,3);
	Vect3 transVelB2(0,0,0);
	Vect3 rotVelB2(0,0,0);
		Vect3 b2force(0,0,0);
		Vect3 b2torque(0,0,10);
		Vect3 jntAxis(0,1,0);

	BodyRigid *body1 = mymodel3.addBodyRigid("body1",body1mass,eye,initialBody1Location,eye,transVelB1,rotVelB1);
	BodyRigid *body2 = mymodel3.addBodyRigid("body2",body2mass,body2I,initialBody2Location,eye,transVelB2,rotVelB2);

	mymodel3.addJointRevolute("revjnt2",body1,Vect3(0,0,0.0),body2,jntAxis);
    Force1Body *b1f3 = mymodel3.addForce1Body("b1f3",body2,b2force,Vect3(0,0,0),true,b2torque,false);

	/////////////model2
	Model2 mymodel2;
    BodyRigid *b1 = mymodel2.addBodyRigid("b1",0,zero,eye,false);
	BodyRigid *b2 = mymodel2.addBodyRigid("b2",0,zero,eye,false);
	BodyRigid *b3 = mymodel2.addBodyRigid("b3",0,zero,eye,false);
	BodyRigid *b4 = mymodel2.addBodyRigid("b4",0,zero,eye,false);
	BodyRigid *b5 = mymodel2.addBodyRigid("b5",0,zero,eye,false);
	BodyRigid *body11 = mymodel2.addBodyRigid("body11",body1mass,eye,eye,false);
	BodyRigid *body12 = mymodel2.addBodyRigid("body12",body2mass,body2I,eye,false);
	BodyRigid *irf = mymodel2.addBodyRigid("irf",0,zero,eye,true);
    JointTranslational *j1 = mymodel2.addJointTranslational("j1",irf,initialBody1Location,b1,Vect3(0,0,0),Vect3(1,0,0),0,transVelB1.x());
	JointTranslational *j2 = mymodel2.addJointTranslational("j2",b1,Vect3(0,0,0),b2,Vect3(0,0,0),Vect3(0,1,0),0,transVelB1.y());
	JointTranslational *j3 = mymodel2.addJointTranslational("j3",b2,Vect3(0,0,0),b3,Vect3(0,0,0),Vect3(0,0,1),0,transVelB1.z());
	JointRevolute *j4 = mymodel2.addJointRevolute("j4",b3,Vect3(0,0,0),b4,Vect3(0,0,0),Vect3(1,0,0),0,rotVelB1.x());
	JointRevolute *j5 = mymodel2.addJointRevolute("j5",b4,Vect3(0,0,0),b5,Vect3(0,0,0),Vect3(0,1,0),0,rotVelB1.y());
	JointRevolute *j6 = mymodel2.addJointRevolute("j6",b5,Vect3(0,0,0),body11,Vect3(0,0,0),Vect3(0,0,1),0,rotVelB1.z());
    JointRevolute *j7 = mymodel2.addJointRevolute("j7",body11,Vect3(0,0,0.0),body12,Vect3(0,0,-0),jntAxis,0);
	Force1Body *b1f = mymodel2.addTorqueOnBody("b1f",body12,b2torque,true);


	mymodel2.buildTree();
	/////////////model1
	double time = 0.0;
	double dt = 0.0005;
	double endtime = 2;

	Vect3 p31, p11, p32, p12;
	Quat q31, q11, q32, q12;
	Vect3 avel3a, avel1a, avel3b, avel1b;
	while (time < endtime)
	{

		mymodel3.calcIndependentStates();

        //std::cout << "time: " << time << std::endl;
        p31 = body1->m_pos;
        //std::cout << "Model3 body1 pos: " << p31.x() << " " << p31.y()<< " " << p31.z() << std::endl;
        p11 = body11->m_pos;
        //std::cout << "Model1 body1 pos: " << p11.x() << " " << p11.y()<< " " << p11.z()  << std::endl;

        p32 = body2->m_pos;
        //std::cout << "Model3 body2 pos: " << p32.x() << " " << p32.y()<< " " << p32.z()  << std::endl;
        p12 = body12->m_pos;
        //std::cout << "Model1 body2 pos: " << p12.x() << " " << p12.y()<< " " << p12.z()  << std::endl;

        q31 = body1->m_q;
		std::cout << "Model3 body1 Quaternion: " << q31.w() << ", " << q31.x() << ", " << q31.y() << ", " << q31.z() << std::endl;
        q11 = body11->m_q;
		//std::cout << "Model2 body1 Quat: " << q11.w() << ", " << q11.x() << ", " << q11.y() << ", " << q11.z() << std::endl;

		q32 = body2->m_q;
		//std::cout << "Model3 body2 Quat: " << q32.w() << ", " << q32.x() << ", " << q32.y() << ", " << q32.z() << std::endl;
        q12 = body12->m_q;
		//std::cout << "Model1 body2 Quat: " << q12.w() << ", " << q12.x() << ", " << q12.y() << ", " << q12.z() << std::endl;

		avel3a = body1->m_wl;
		//std::cout << "Model3 body 1 Ang Vel: " << avel3a.x() << " , "<< avel3a.y() << " , "<< avel3a.z() << std::endl;
		avel1a = body11->m_wl;
		//std::cout << "Model1 body 1 Ang Vel: " << avel1a.x() << " , "<< avel1a.y() << " , "<< avel1a.z() << std::endl;

		avel3b = body2->m_wl;
		//std::cout << "Model3 body 2 Ang Vel: " << avel3b.x() << " , "<< avel3b.y() << " , "<< avel3b.z() << std::endl;
		avel1b = body12->m_wl;
		//std::cout << "Model1 body 2 Ang Vel: " << avel1b.x() << " , "<< avel1b.y() << " , "<< avel1b.z() << std::endl;

        if ( time > 0.6781)
        {
            int crap =1;
        }

		mymodel3.integrate(time,time+dt,true);
		mymodel2.integrate(time,time+dt,true);
		time = time + dt;
	}

		// This is the test to see if something has changed //////////////////////////////////////////////////////////////////
		// Known answers
		Vect3 knownpb1(1,2,3);
		Vect3 knownpb2(1,2,3);
        Quat knownqb1(-0.801144, 0, 0, 0.598472);
		Quat knownqb1b(0.801144, 0, 0, -0.598472);
		Quat knownqb2(-0.801144, 0, 0, 0.598472);
		Quat knownqb2b(0.801144, 0, 0, -0.598472);
		Vect3 knownaa(0,0,5);

		Vect3 errorp1= knownpb1-p31;
		Vect3 errorp1b = knownpb1 - p11;
		Vect3 errorp2= knownpb2-p32;
		Vect3 errorp2b = knownpb2-p12;
		double errorqb1w = knownqb1.w()-q31.w();
		double errorqb1x = knownqb1.x()-q31.x();
		double errorqb1y = knownqb1.y()-q31.y();
		double errorqb1z = knownqb1.z()-q31.z();

		double errorqb1wb = knownqb1b.w()-q11.w();
		double errorqb1xb = knownqb1b.x()-q11.x();
		double errorqb1yb = knownqb1b.y()-q11.y();
		double errorqb1zb = knownqb1b.z()-q11.z();

		double errorqb2w = knownqb2.w()-q32.w();
		double errorqb2x = knownqb2.x()-q32.x();
		double errorqb2y = knownqb2.y()-q32.y();
		double errorqb2z = knownqb2.z()-q32.z();

		double errorqb2wb = knownqb2b.w()-q12.w();
		double errorqb2xb = knownqb2b.x()-q12.x();
		double errorqb2yb = knownqb2b.y()-q12.y();
		double errorqb2zb = knownqb2b.z()-q12.z();
		
		Vect3 erroraab1= knownaa-avel3a;
		Vect3 erroraab1b = knownaa-avel1a;
		Vect3 erroraab2 = knownaa-avel3b;
		Vect3 erroraab2b = knownaa-avel1b;
		//std::cout << "Error: " << errorp1.x() << ", " << errorp1.y() << ", " << errorp1.z() << std::endl;
		//std::cout << "Error: " << errorp1b.x() << ", " << errorp1b.y() << ", " << errorp1b.z() << std::endl;
		//std::cout << "Error: " << errorp2.x() << ", " << errorp2.y() << ", " << errorp2.z() << std::endl;
		//std::cout << "Error: " << errorp2b.x() << ", " << errorp2b.y() << ", " << errorp2b.z() << std::endl;
		//std::cout << "Error: " << errorqb1w << ", " << errorqb1x << ", " << errorqb1y << ", "<< errorqb1z  << std::endl;
		//std::cout << "Error: " << errorqb1wb << ", " << errorqb1xb << ", " << errorqb1yb << ", "<< errorqb1zb  << std::endl;
		//std::cout << "Error: " << errorqb2w << ", " << errorqb2x << ", " << errorqb2y << ", "<< errorqb2z  << std::endl;
		//std::cout << "Error: " << errorqb2wb << ", " << errorqb2xb << ", " << errorqb2yb << ", "<< errorqb2zb  << std::endl;
		//std::cout << "Error: " << erroraab1.x() << ", " << erroraab1.y() << ", " << erroraab1.z() << std::endl;
		//std::cout << "Error: " << erroraab1b.x() << ", " << erroraab1b.y() << ", " << erroraab1b.z() << std::endl;
		//std::cout << "Error: " << erroraab2.x() << ", " << erroraab2.y() << ", " << erroraab2.z() << std::endl;
		//std::cout << "Error: " << erroraab2b.x() << ", " << erroraab2b.y() << ", " << erroraab2b.z() << std::endl;
		
		if ((fabs(errorp1.x()) > 0.000005) | (fabs(errorp1.y()) > 0.000005) | (fabs(errorp1.z()) > 0.000005) | 
			(fabs(errorp1b.x()) > 0.000005) | (fabs(errorp1b.y()) > 0.000005) | (fabs(errorp1b.z()) > 0.000005) | 
			(fabs(errorp2.x()) > 0.000005) | (fabs(errorp2.y()) > 0.000005) | (fabs(errorp2.z()) > 0.000005) |
			(fabs(errorp2b.x()) > 0.000005) | (fabs(errorp2b.y()) > 0.000005) | (fabs(errorp2b.z()) > 0.000005) |
			(fabs(errorqb1w) > 0.0000005) | (fabs(errorqb1x) > 0.0000005) | (fabs(errorqb1y) > 0.0000005) | (fabs(errorqb1z)>0.0000005 ) |
			(fabs(errorqb1wb) > 0.0000005) | (fabs(errorqb1xb) > 0.0000005) | (fabs(errorqb1yb) > 0.0000005) | (fabs(errorqb1zb)>0.0000005 ) |
			(fabs(errorqb2w) > 0.0000005) | (fabs(errorqb2x) > 0.0000005) | (fabs(errorqb2y) > 0.0000005) | (fabs(errorqb2z)>0.0000005 ) |
			(fabs(errorqb2wb) > 0.0000005) | (fabs(errorqb2xb) > 0.0000005) | (fabs(errorqb2yb) > 0.0000005) | (fabs(errorqb2zb)>0.0000005 ) |
			(fabs(erroraab1.x()) > 0.0000005) | (fabs(erroraab1.y()) > 0.0000005) | (fabs(erroraab1.z()) > 0.0000005) |
			(fabs(erroraab1b.x()) > 0.0000005) | (fabs(erroraab1b.y()) > 0.0000005) | (fabs(erroraab1b.z()) > 0.0000005) |
			(fabs(erroraab2.x()) > 0.0000005) | (fabs(erroraab2.y()) > 0.0000005) | (fabs(erroraab2.z()) > 0.0000005) |
			(fabs(erroraab2b.x()) > 0.0000005) | (fabs(erroraab2b.y()) > 0.0000005) | (fabs(erroraab2b.z()) > 0.0000005) )
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
