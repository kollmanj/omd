#ifndef SWIG	// don't include in SWIG
#include "ForceCollisionDynWorld.h"
//#include "OMDMatrix3.h"

#include <iostream>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>


using namespace std;

namespace OMD
{

	ForceCollisionDynWorld::ForceCollisionDynWorld ( string const& name, double stiff, double damp, double frict, double thresh ): Force(name),
		mk(stiff),
		md(damp),
		m_mu(frict),
		m_thresh(thresh)
	{
		//Bullet initialisation.
		mBroadphase = new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024);
		mCollisionConfig = new btDefaultCollisionConfiguration();
		mDispatcher = new btCollisionDispatcher(mCollisionConfig);
		mSolver = new btSequentialImpulseConstraintSolver();
		m_btDDynamicsWorld = new btDiscreteDynamicsWorld (mDispatcher, mBroadphase, mSolver, mCollisionConfig);

		//// write to a file to debug
	    fout = ofstream("afile.txt");
	}

	ForceCollisionDynWorld::~ForceCollisionDynWorld()
	{
		// file to write to for debugging
		fout.close();

		// if collision world provided some pointer will point to nothing
		if(mBroadphase)
		{
			delete mBroadphase;
			mBroadphase = 0;
		}
		if(mCollisionConfig)
		{
			delete mCollisionConfig;
			mCollisionConfig = 0;
		}
		if (mDispatcher)
		{
			delete mDispatcher;
			mDispatcher = 0;
		}
		if (mSolver)
		{
			delete mSolver;
			mSolver=0;
		}
		//// TODO:  See if I should do this and not the above
		// Seems like I should have to do this but causes error from python
		//if (m_btCollisionWorld)
		//{
		//	delete m_btCollisionWorld;
		//	m_btCollisionWorld = 0;
		//}
	}


	void ForceCollisionDynWorld::apply(double t)
	{
		btCollisionObjectArray &objectArray = m_btDDynamicsWorld->getCollisionObjectArray();
		// loop through and position collision objects in the collision world
		for (int i=0; i < objectArray.size(); i++)
		{
			btCollisionObject *object = objectArray[i];
			if (object->getUserPointer()) //
			{
				BodyRigid * b = static_cast<BodyRigid *>(object->getUserPointer());
				Vect3 pos = b->m_pos;										// Get OMD Position
				Quat OMDq = b->m_q;											// Get OMD Quaternion
				btVector3 btpos(pos.x(),pos.y(),pos.z());					// Set Bullet Position
				btQuaternion btquat(OMDq.x(),OMDq.y(),OMDq.z(),OMDq.w());	// Set Bullet Quaternion
				btTransform bttrans(btquat,btpos);		
				object->setWorldTransform(bttrans);
			}
		}

		m_btDDynamicsWorld->performDiscreteCollisionDetection();
		int numManifolds = m_btDDynamicsWorld->getDispatcher()->getNumManifolds();

		for (int i=0;i<numManifolds;i++)
		{
			btPersistentManifold* contactManifold = m_btDDynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			const btCollisionObject* obA = contactManifold->getBody0();
			const btCollisionObject* obB = contactManifold->getBody1();
			BodyRigid * bA = static_cast<BodyRigid *>(obA->getUserPointer());
			BodyRigid * bB = static_cast<BodyRigid *>(obB->getUserPointer());

			//TIRE STUFF BEGIN 
			// check to see if bodies are tires
			bool AisTire = false;
			bool BisTire = false;
			bool AisGrnd = false;
			bool BisGrnd = false;
			if (mTireBodySlipCurveMap.find(bA)!=mTireBodySlipCurveMap.end())
			{
				AisTire = true;
				std::cout << "A is Tire" << std::endl;
			}
			if (mTireBodySlipCurveMap.find(bB)!=mTireBodySlipCurveMap.end())
			{
				BisTire = true;
			}
			if ( bA )
			{
				if(bA->isFixed())
				{
					AisGrnd = true;
					std::cout << "A is Ground (cause it is fixed)!!!" << std::endl;
				}
			}
			else	// bA is not associated with OMD Body
			{
				AisGrnd = true;
			}

			if ( bB )
			{
				if(bB->isFixed())
				{
					BisGrnd = true;
				}
			}
			else	// bB is not associated with OMD Body
			{
				BisGrnd = true;
			}

			bool tireGrndInteraction = false;
			if ( (AisTire && BisGrnd) || (BisTire && AisGrnd) )
			{
				tireGrndInteraction = true;
			}

			bool nontirenongrndInteraction = false;
			if (( !AisTire && !BisTire) )	// Niether one is a tire
			{
				nontirenongrndInteraction = true;
			}
			//// Now we have determined the two things in contact and what they are

			//// write to a file to debug
				fout << "Body A: "<< bA->m_name << endl;
				if (bB)
				{
					fout << "Body B: "<< bB->m_name << endl;
				}
			//// DONE debug

			int numContacts = contactManifold->getNumContacts();
			/// write to file for debug
			fout << "numContacts: "<< numContacts << endl;
			//// DONE debug
			// velocity of the contact point on body A, fill with 0 in case not associated with a body
			Vect3 cntctPntBAVelGlobal(0,0,0);
			// velocity of the contact point on body B, fill with 0 in case not associated with a body
			Vect3 cntctPntBBVelGlobal(0,0,0);


				for (int j=0;j<numContacts;j++)
				{
					btManifoldPoint& pt = contactManifold->getContactPoint(j);
					btScalar distance = pt.getDistance();

					if (distance<0.f)
					{
						Vect3 fn,tn,ff,tf;
						// There is contact so find the relative distance between points and relative speeds
						const btVector3& ptA = pt.m_localPointA;
						const btVector3& normalOnB = pt.m_normalWorldOnB;	// normal vector
						
						OMD::Vect3 ptALocal(ptA.x(),ptA.y(),ptA.z());

						const btVector3& ptB = pt.m_localPointB;
						OMD::Vect3 ptBLocal(ptB.x(),ptB.y(),ptB.z());

						//OMD::Vect3 temp = ptALocal.normalized();
						// reduce ptLocal by distance
						//ptALocal = ptALocal + temp*distance;
						OMD::Vect3 PtAVelGlobal(0,0,0);
						if (bA) // if body A is an OMD body it may have a velocity 
						{
							PtAVelGlobal = bA->getVelocityGlobal(ptALocal);
						}

						OMD::Vect3 PtBVelGlobal(0,0,0);
						if (bB) // if body A is an OMD body it may have a velocity 
						{
							PtBVelGlobal = bB->getVelocityGlobal(ptBLocal);
						}
						Vect3 RelVel = PtAVelGlobal - PtBVelGlobal;
						double PtVelInNorm = RelVel.dot(Vect3(normalOnB.x(),normalOnB.y(),normalOnB.z()));



		Quat BodyQuat = bA->m_q;

		double stiff = mk*distance;
		double damp = md*PtVelInNorm;
		// allow for specific stiff and damp
		if (bA && bB)
		{
			if (bA->m_stiff > 0)
			{
				    if (abs(distance) > 0.01)
					{
						distance = -0.01;
					}
					stiff = bA->m_stiff*distance;
					damp = bA->m_damp*PtVelInNorm;
					
			}
		}
		if (bB && bA)  // TODO, now B overrides A if they both exist
		{
			if (bB->m_stiff > 0)
			{
					if (abs(distance) > 0.01)
					{
						distance = -0.01;
					}
					stiff = bB->m_stiff*distance;
					damp = bB->m_damp*PtVelInNorm;
			}

		}
		
		Vect3 dampVG(0,0,0);

		Vect3 normG(normalOnB.x(),normalOnB.y(),normalOnB.z());
		if (damp < 0) 
		{
			dampVG = normG*-damp;
		}
		
		Vect3 stiffVG = normG*-stiff;
		// total force in global
		Vect3 fTotalVG = dampVG+ stiffVG;
		fn = BodyQuat.inverse()*fTotalVG;
		tn = ptALocal.cross(fn);
		OMD::Vect3 fg =normG;

						// vector normal to both normal and relitive velocity
						OMD::Vect3 tempVect = RelVel.cross(Vect3(normalOnB.x(),normalOnB.y(),normalOnB.z()));
						OMD::Vect3 PtVelTangential = tempVect.cross(Vect3(normalOnB.x(),normalOnB.y(),normalOnB.z()));
						ff = PtVelTangential.normalized()* m_mu*fTotalVG.norm();

//						calcNormalForceAndTorque(bA,ptALocal,distance,Vect3(normalOnB.x(),normalOnB.y(),normalOnB.z()),fn,tn);
		
						cntctPntBAVelGlobal = bA->getVelocityGlobal(ptALocal);
						if (bB)
						{
							const btVector3& ptB = pt.m_localPointB;
							// reduce ptLocal by distance
							OMD::Vect3 ptBLocal(ptB.x(),ptB.y(),ptB.z());
							OMD::Vect3 temp = ptBLocal;
							temp.normalize();
							ptBLocal = ptBLocal + temp*distance;
							cntctPntBBVelGlobal = bB->getVelocityGlobal(ptBLocal);
						}
						else
						{
							cntctPntBBVelGlobal = Vect3(0,0,0);
						}

						if (tireGrndInteraction)
						{
							doTireFrictionForceAndTorque(bA, ptALocal,(cntctPntBAVelGlobal-cntctPntBBVelGlobal), fn, ff, tf);
						}
						else
						{
							// reduce ptLocal by distance
							OMD::Vect3 ptLocal(ptA.x(),ptA.y(),ptA.z());
							OMD::Vect3 temp = ptLocal.normalized();
							//temp.normalize();
							ptLocal = ptLocal + temp*distance;
							calcFrictionForceAndTorque(bA, ptALocal,(cntctPntBAVelGlobal-cntctPntBBVelGlobal), fn, ff, tf);
						}
					//	if (tireGrndInteraction || nontirenongrndInteraction)
					//	{
							if (bA)
							{
							bA->forceAndTorqueAccum(fn+ff,true,Vect3(0,0,0),tn+tf,true);
							//bA->forceAccumGlobal(fTotalVG+ff,cntctPntBBVelGlobal);
							}
							if (bB)
							{
								bB->forceAndTorqueAccum(fn+ff,true,Vect3(0,0,0),tn+tf,true);
								//bB->forceAccumGlobal(fTotalVG+ff,cntctPntBBVelGlobal);
							}
					//	}

					}
				}
			

		}
		
		return;
	}

	void ForceCollisionDynWorld::calcNormalForceAndTorque(BodyRigid *b, Vect3 ptLocal, double dist, Vect3 normG, Vect3 &f, Vect3 &t)
	{
		Quat BodyQuat = b->m_q;

		Vect3 PtVelGlobal = b->getVelocityGlobal(ptLocal);
		double PtVelInNorm = PtVelGlobal.dot(normG);
		double damp = md*PtVelInNorm;
		Vect3 dampVG(0,0,0);
		if (damp < 0) 
		{
			dampVG = normG*-damp;
		}
		// TODO: mk is for all collision now but allow for something in body or bodies
		double stiff = mk*dist;
		Vect3 stiffVG = normG*-stiff;
		// total force in global
		Vect3 fTotalVG = dampVG+ stiffVG;
		f = BodyQuat.inverse()*fTotalVG;
		t = ptLocal.cross(f);
		OMD::Vect3 fg =normG;
		return;
	}

	void ForceCollisionDynWorld::calcFrictionForceAndTorque(BodyRigid *b, Vect3 ptLocal,Vect3 ptRelativeVelGlobal, Vect3 normalF, Vect3 &f, Vect3 &t)
	{
		Mat3x3 B2W = b->getRot();
		//std::cout << "Body: " << b->GetName() << std::endl;
		//std::cout << B2W[0][0] << ", " << B2W[0][1]  << ", " << B2W[0][2]<< std::endl;
		//std::cout << B2W[1][0] << ", " << B2W[1][1]  << ", " << B2W[1][2]<< std::endl;
		//std::cout << B2W[1][0] << ", " << B2W[1][1]  << ", " << B2W[1][2]<< std::endl;
		//std::cout << "Body: " << b->GetName() << std::endl;
		//Mat3x3 W2B = B2W.getTranspose();
		Mat3x3 W2B = B2W.transpose();
		Vect3 bVelLocal = W2B *ptRelativeVelGlobal;
		//std::cout << "Body : " << b->GetName() <<" Global Vel" << ", "<< ptRelativeVelGlobal.x <<", " << ptRelativeVelGlobal.y << ", "<<ptRelativeVelGlobal.z << std::endl;
		//std::cout << "Body : " << b->GetName() << " Local Vel " <<", "<< bVelLocal.x <<", " << ", "<< bVelLocal.y << ", "<<bVelLocal.z << std::endl;

		double nForceMag = normalF.norm();
		Vect3 norm = normalF.normalized();
		double VInNorm = norm.dot(bVelLocal);
		Vect3 bVelTang = bVelLocal - norm*VInNorm;
		double speed = bVelTang.norm();
		//speed = 1;
		bVelTang.normalize();
		//double thresh = 0.0001;
		if (fabs(speed) > m_thresh)
		{
			f = bVelTang*(-m_mu*nForceMag);
			//std::cout << "Body : " << b->GetName() <<", "<< f.x <<", " << f.y << ", "<<f.z << std::endl;
		
		t = ptLocal.cross(f);
		//t = Vect3(0,0,0);

		}
		else
		{
			f = Vect3(0,0,0);
			t = Vect3(0,0,0);
		}

	}

	void ForceCollisionDynWorld::addTire(double radius, double width, Vect3 offset, Mat3x3 rot, BodyRigid *body,  OMD::Curve2DLinInterp slipCurve)
	{
		btCylinderShape *cylinder = new btCylinderShape(btVector3(radius,width/2,radius));
		cylinder->setMargin(0.001);
		addShape(cylinder,offset,rot,body);
		//mTireBodySlipCurveMap.insert(pair<Body *, Curve2DLinInterp> (body,slipCurve));
		mTireBodySlipCurveMap[body] = slipCurve;

		return;
		//mTireBodies.push_back(body);
	}

	void ForceCollisionDynWorld::addBox(double x, double y, double z, Vect3 offset, Mat3x3 rot, BodyRigid *body)
	{
		btBoxShape *box = new btBoxShape(btVector3(x,y,z));
		box->setMargin(0.001);
		addShape(box,offset,rot,body);
		return;
	}

	void ForceCollisionDynWorld::addCapsule(double radius, double height, Vect3 offset, Mat3x3 rot, BodyRigid * body)
	{
		btCapsuleShape *capsule = new btCapsuleShape(radius, height);
		capsule->setMargin(0.001);
		addShape(capsule,offset,rot,body);
		return;
	}

	void ForceCollisionDynWorld::addCylinder(double radius, double width, Vect3 offset, Mat3x3 rot, BodyRigid * body)
	{
		btCylinderShape *cylinder = new btCylinderShape(btVector3(radius,width,radius));
		cylinder->setMargin(0.001);
		addShape(cylinder,offset,rot,body);
		return;
	}

	void ForceCollisionDynWorld::addCompoundShape(btCompoundShape *shape, BodyRigid *body)
	{
		shape->setMargin(0.001);
		btDefaultMotionState* MotionState = new btDefaultMotionState();
		btRigidBody *btBody(0);
		if (body)
		{
			btBody = new btRigidBody(1,MotionState,shape,btVector3(0,0,0));
			btBody->setUserPointer(body);
		}
		else
		{
			btBody = new btRigidBody(0,MotionState,shape,btVector3(0,0,0));
			// Not associated with a body set it's pointer to 0
			btBody->setUserPointer(0);
			// No rotation positioned at 0,0,0 
			//btTransform collisionTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
			//collisionObject->setWorldTransform(collisionTransform);
		}
		//m_btDDynamicsWorld->addCollisionObject(collisionObject);
		m_btDDynamicsWorld->addRigidBody(btBody);
	}

	void ForceCollisionDynWorld::addShape(btCollisionShape* shape, Vect3 offset, Mat3x3 rot, BodyRigid * body)
	{
		Quat OMDq(rot);
		// check to see if we already have a btCompoundShape to which we can add the shape
		/*		btCollisionObjectArray &objectArray = m_btDDynamicsWorld->getCollisionObjectArray();
		btCompoundShape *cShape(0);
		// loop through and position collision objects in the collision world
		for (int i=0; i < objectArray.size(); i++)
		{
		btCollisionObject *object = objectArray[i];
		BodyRigid * b = static_cast<BodyRigid *>(object->getUserPointer());
		if (body == b)
		{
		//btCollisionShape * ctest = object->getCollisionShape();
		cShape = dynamic_cast<btCompoundShape*>(object->getCollisionShape());
		//btMatrix3x3 btrot(rot[0][0],rot[0][1],rot[0][2],rot[1][0],rot[1][1],rot[1][2],rot[2][0],rot[2][1],rot[2][2]);
		btQuaternion btquat(OMDq.x(),OMDq.y(),OMDq.z(),OMDq.w());
		//btrot.getRotation(btquat);

		btTransform shapeTrans(btquat,btVector3(offset.x(),offset.y(),offset.z()));
		cShape->addChildShape(shapeTrans,shape);
		break;
		}
		}
		// if we didn't find a shape already associated with a body make one
		if (!cShape)
		{*/
		btCompoundShape * cShape = new btCompoundShape();

		//btMatrix3x3 btrot(rot[0][0],rot[0][1],rot[0][2],rot[1][0],rot[1][1],rot[1][2],rot[2][0],rot[2][1],rot[2][2]);
		btQuaternion btquat(OMDq.x(),OMDq.y(),OMDq.z(),OMDq.w());
		//btrot.getRotation(btquat);

		btTransform shapeTrans(btquat,btVector3(offset.x(),offset.y(),offset.z()));
		cShape->addChildShape(shapeTrans,shape);
		cShape->setMargin(0.001);
		btDefaultMotionState* MotionState = new btDefaultMotionState();

		//btCollisionObject *collisionObject = new btCollisionObject();
		//collisionObject->setCollisionShape(shape);
		btRigidBody *btBody(0);
		if (body)
		{
			btBody = new btRigidBody(1,MotionState,cShape,btVector3(0,0,0));
			btBody->setUserPointer(body);
		}
		else
		{
			// Not associated with a body set it's pointer to 0
			btBody = new btRigidBody(0,MotionState,cShape,btVector3(0,0,0));
			btBody->setUserPointer(0);
			// No rotation positioned at 0,0,0 
			//btTransform collisionTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
			//collisionObject->setWorldTransform(collisionTransform);
		}
		//m_btDDynamicsWorld->addCollisionObject(collisionObject);
		m_btDDynamicsWorld->addRigidBody(btBody);
		// }
		return;
	}
	void ForceCollisionDynWorld::doTireFrictionForceAndTorque(BodyRigid *b, Vect3 ptLocal,Vect3 ptRelativeVelGlobal,Vect3 normalF, Vect3 &f, Vect3 &t)
	{
		Vect3 vel = b->getVelocityGlobal();
		double slip = (ptRelativeVelGlobal.norm())/(vel.norm());
		//std::cout << "slip: " << slip << std::endl;
		OMD::Curve2DLinInterp slipCurve = mTireBodySlipCurveMap[b];
		double mu = slipCurve.getDependent(slip);
		Mat3x3 B2W = b->getRot();
		Mat3x3 W2B = B2W.transpose();
		Vect3 bVelLocal = W2B *ptRelativeVelGlobal;
		//double nForceMag = normalF.normalize();
		double nForceMag = normalF.norm();
		Vect3 norm = normalF.normalized();
		double VInNorm = norm.dot(bVelLocal);
		Vect3 bVelTang = bVelLocal - norm*VInNorm;
		double speed = bVelTang.norm();
		bVelTang.normalize();
		f = bVelTang*(-mu*nForceMag);
		t = ptLocal.cross(f);
		//      std::cout << "norm x: " << norm.x << " norm y: "<< norm.y << " norm z: " << norm.z << std::endl;
		//      std::cout << "Tang x: " << bVelTang.x << " Tang y: "<< bVelTang.y << " Tang z: " << bVelTang.z << std::endl;
		//      std::cout << "pt x: " << ptLocal.x << " pt y: "<< ptLocal.y << " pt z: " << ptLocal.z << std::endl;
		return;
	}
}
#endif
