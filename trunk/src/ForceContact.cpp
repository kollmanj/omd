/***************************************************************************
 *   Copyright (C) 2008 by John Kollman,,,                                 *
 *   opensourcemultibodydynamics@gmail.com                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "ForceContact.h"
#include "BodyRigid.h"
#include <iostream>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
namespace OMD
{
	ForceContact::ForceContact ( string const& name, btCollisionObjectArray objects, double stiff, double damp, double frict, double thresh  ) : Force(name),
		mk(stiff),
		md(damp),
		m_mu(frict),
		m_thresh(thresh)
	{
		//Bullet initialisation.
		mBroadphase = new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024);
		//mBroadphase = new btAxisSweep3(btVector3(-10000000,-10000000,-10000000), btVector3(10000000,10000000,10000000), 16384);
		mCollisionConfig = new btDefaultCollisionConfiguration();
//		mCollisionConfig->setConvexConvexMultipointIterations();
		mDispatcher = new btCollisionDispatcher(mCollisionConfig);
		m_btCollisionWorld = new btCollisionWorld (mDispatcher, mBroadphase, mCollisionConfig);


		// Add shapes to the collision World
		for ( int i=0 ; i < objects.size(); i++ )
		{
			btCollisionObject *object = objects[i];
			BodyRigid * b = static_cast<BodyRigid *>(object->getUserPointer());
			std::string bodyName = b->m_name;
			//m_btCollisionWorld->addCollisionObject(object, btBroadphaseProxy::DebrisFilter, btBroadphaseProxy::StaticFilter);
			m_btCollisionWorld->addCollisionObject(object);
			std::cout << "Adding Collision Object to Body: " <<bodyName << std::endl;
		}
		//		mk = 2000;
		//		md = 100;
		//		m_mu = 0.1;
		collisionMargin = 0.001;
		collisionWorldScale= 1;
	}

	ForceContact::ForceContact ( string const& name, btCollisionWorld *collisionWorld, double stiff, double damp, double frict, double thresh): Force(name),
		mk(stiff),
		md(damp),
		m_mu(frict),
		m_thresh(thresh),
		m_btCollisionWorld(collisionWorld)
	{
		// nothing to be done the collision World defined and given to us
		mBroadphase=0;
		mCollisionConfig=0;
		mDispatcher=0;
		//		mk = 3000;
		//		md = 100;
		//		m_mu = 0.1;
		collisionMargin = 0.001;
		collisionWorldScale=1.0;
	}
	ForceContact::ForceContact ( string const& name, double stiff, double damp, double frict, double thresh ): Force(name),
		mk(stiff),
		md(damp),
		m_mu(frict),
		m_thresh(thresh)
	{
		//Bullet initialisation.
		//mBroadphase = new btAxisSweep3(btVector3(-10000,-10000,-10000), btVector3(10000,10000,10000), 1024);
		mBroadphase = new btAxisSweep3(btVector3(-10000000,-10000000,-10000000), btVector3(10000000,10000000,10000000), 16384);
		mCollisionConfig = new btDefaultCollisionConfiguration();
//		mCollisionConfig->setConvexConvexMultipointIterations();
		mDispatcher = new btCollisionDispatcher(mCollisionConfig);
		m_btCollisionWorld = new btCollisionWorld (mDispatcher, mBroadphase, mCollisionConfig);
		collisionMargin = 0.001;
		collisionWorldScale=1.0;
	}

	ForceContact::~ForceContact()
	{
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
		// Seems like I should have to do this but causes error from python
		//if (m_btCollisionWorld)
		//{
		//	delete m_btCollisionWorld;
		//	m_btCollisionWorld = 0;
		//}
	}


	btCollisionWorld* ForceContact::getbtCollisionWorld()
	{
		return m_btCollisionWorld;
	}

	void ForceContact::apply(double t)
	{
	    m_c.clear();  // clear storage of contact information
        relocateContactObjects();
		//int numCollisionObjects = m_btCollisionWorld->getNumCollisionObjects();
		m_btCollisionWorld->performDiscreteCollisionDetection();
		int numManifolds = m_btCollisionWorld->getDispatcher()->getNumManifolds();
		
		//std::cout << "numManifolds: " << numManifolds << std::endl;

		for (int i=0;i<numManifolds;i++)
		{
			btPersistentManifold* contactManifold = m_btCollisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
			//btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
			const btCollisionObject* obA = contactManifold->getBody0();
			//btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
			const btCollisionObject* obB = contactManifold->getBody1();
			BodyRigid * bA = static_cast<BodyRigid *>(obA->getUserPointer());
			BodyRigid * bB = static_cast<BodyRigid *>(obB->getUserPointer());
			int numContacts = contactManifold->getNumContacts();

			/// TIRE STUFF BEGIN ///
			// see if body A is a Tire
			bool bAisTire = false;
			vector<OMD::BodyRigid *>::iterator it;
			for ( it=mTireBodies.begin() ; it < mTireBodies.end(); it++ )
			{
				if (bA==(*it) && (!bB || bB->isFixed()) )
				{
					bAisTire = true;
					break;
				}
			}
			// see if body B is a Tire
			bool bBisTire = false;
			//vector<OMD::Body *>::iterator it;
			for ( it=mTireBodies.begin() ; it < mTireBodies.end(); it++ )
			{
				if (bB==(*it) && (!bA || bA->isFixed()) )
				{
					bAisTire = true;
					break;
				}
			}
			/// TIRE STUFF END ///
			// velocity of the contact point on body A, fill with 0 in case not associated with a body
			Vect3 cntctPntBAVelGlobal(0,0,0);
			// velocity of the contact point on body B, fill with 0 in case not associated with a body
			Vect3 cntctPntBBVelGlobal(0,0,0);

			if (bA || bB)  // if two contact objects with no bodies associated ignore
			{
			    for (int j=0; j<numContacts; j++)
			    {
					btManifoldPoint& pt = contactManifold->getContactPoint(j);
//					std::cout << "pt: " << pt.x() << ", " << pt.y() ", " << pt.z() << std::endl;
					btScalar distance = pt.getDistance();
					if (distance<0.f)
					{
					    const btVector3& pos = pt.getPositionWorldOnA();
					    Vect3 pntV(pos.x(),pos.y(),pos.z());
					    pntV /= collisionWorldScale;   // shouldn't have to do this but can't get rid of 0.08 contact margin
//					    std::cout << "pos: "<<pos.x()<<", "<< pos.y()<<", " << pos.z()  << std::endl;
					    const btVector3& norm = pt.m_normalWorldOnB;
//					    std::cout << "norm: "<< norm.x()<<", "<< norm.y()<<", " << norm.z()  << std::endl;
					    Vect3 normV(-norm.x(),-norm.y(),-norm.z());
					    // velocity of pnt on A
					    Vect3 bAPntDot(0,0,0);
					    // velcotiy of pnt on B
					    Vect3 bBPntDot(0,0,0);
					    if (bA)
					    {
					        Vect3 bAPntLocal = bA->getPointInLocal(pntV);
					        bAPntDot  = bA->getVelocityGlobal(bAPntLocal);
//					        std::cout << "Body: " << bA->getName() << " PntVel: " << bAPntDot << std::endl;
					    }

					    if (bB)
					    {
					        Vect3 bBPntLocal = bB->getPointInLocal(pntV);
					        bBPntDot = bB->getVelocityGlobal(bBPntLocal);
//					        std::cout << "Body: " << bB->getName() << " PntVel: " << bBPntDot << std::endl;
					    }

					    // relative velocity of the points
					    Vect3 relVel = bAPntDot - bBPntDot;
					    // relative velocity in the normal direction
					    double relVelN = relVel.dot(normV);
//					    std::cout << "revVelN: " << relVelN << std::endl;
					    // normal force Magnitude on A
					    double normFAMag = distance*mk - relVelN*md;
					    Vect3 normFA = normFAMag*normV;

					    Vect3 tangVelUnit = (relVel - relVelN*normV);
						tangVelUnit.normalize();
					    double speed = sqrt(tangVelUnit.x()*tangVelUnit.x() + tangVelUnit.y()*tangVelUnit.y() + tangVelUnit.z()*tangVelUnit.z());
					    Vect3 frictionFA(0,0,0);
					    if (fabs(speed) > m_thresh)
                        {
                            frictionFA = -1*tangVelUnit*fabs(m_mu*normFAMag);
                        }

                        Vect3 FonA = normFA+frictionFA;
					    if (bA)
					    {
					        bA->forceAccumGlobal(FonA,pntV);
					        storeContactInfo(bA,FonA,pntV);
					        //std::cout << "Body A: " << bA->m_name << std::endl;
					    }
					    if (bB)
					    {
					        bB->forceAccumGlobal(-FonA,pntV);
					        storeContactInfo(bB,-FonA,pntV);
					        //std::cout << "Body B: " << bB->m_name << std::endl;
					    }
//					    std::cout <<"tangVelUnit: " << tangVelUnit << std::endl;
//					    std::cout <<"frictionFA: " << frictionFA << std::endl;

					}

			    }
			}


		}

		return;
	}



	void ForceContact::addTire(BodyRigid *body)
	{
		mTireBodies.push_back(body);
	}

	void ForceContact::addBox(double x, double y, double z, Vect3 offset, Mat3x3 rot, BodyRigid *body)
	{
	    x *= collisionWorldScale;   // shouldn't have to do this but can't get rid of 0.08 contact margin
	    y *= collisionWorldScale;
	    z *= collisionWorldScale;
	    offset *= collisionWorldScale;

		btBoxShape *box = new btBoxShape(btVector3(x,y,z));
		box->setMargin(collisionMargin);
		btCompoundShape *shape = new btCompoundShape();

		btMatrix3x3 btrot(rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
		btQuaternion btquat;
		btrot.getRotation(btquat);

		btTransform shapeTrans(btquat,btVector3(offset.x(),offset.y(),offset.z()));
		shape->addChildShape(shapeTrans,box);
		shape->setMargin(collisionMargin);

		btCollisionObject *collisionObject = new btCollisionObject();
		collisionObject->setCollisionShape(shape);
		if (body)
		{
			collisionObject->setUserPointer(body);
			std::cout << "Collision Shape Attached To: " << body->m_name;
		}
		else
		{
			// Not associated with a body set it's pointer to 0
			collisionObject->setUserPointer(0);
			// No rotation positioned at 0,0,0
			btTransform collisionTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
			collisionObject->setWorldTransform(collisionTransform);
		}
		m_btCollisionWorld->addCollisionObject(collisionObject);
		return;
	}

	void ForceContact::addBox(double x, double y, double z, vector<double> offset, vector<double> rot, BodyRigid *body)
	{
		Vect3 offset_(offset[0],offset[1],offset[2]);
		Mat3x3 rot_;
		rot_ << rot[0], rot[1],	rot[2],
				rot[3],	rot[4],	rot[5],
				rot[6],	rot[7],	rot[8];
		return addBox(x,y,z,offset_,rot_, body);
	}

	void ForceContact::addSphere(double radius, vector<double> offset, vector<double> rot, BodyRigid *body)
	{
		return addCapsule(radius,0.0,offset,rot,body);
	}
	
	void ForceContact::addSphere(double radius, Vect3 offset, Mat3x3 rot, BodyRigid * body)
	{
		return addCapsule(radius,0.0,offset,rot,body);
	}

	void ForceContact::addCapsule(double radius, double height, vector<double> offset, vector<double> rot, BodyRigid *body)
	{
		Vect3 offset_(offset[0],offset[1],offset[2]);
		Mat3x3 rot_;
		rot_ << rot[0], rot[1],	rot[2],
				rot[3],	rot[4],	rot[5],
				rot[6],	rot[7],	rot[8];
		return addCapsule(radius,height,offset_,rot_, body);
	}

	void ForceContact::addCapsule(double radius, double height, Vect3 offset, Mat3x3 rot, BodyRigid * body)
	{
	    radius *= collisionWorldScale;   // shouldn't have to do this but can't get rid of 0.08 contact margin
	    height *= collisionWorldScale;
	    offset *= collisionWorldScale;

		btCapsuleShape *capsule = new btCapsuleShape(radius, height);
		capsule->setMargin(collisionMargin);
		btCompoundShape *shape = new btCompoundShape();

		btMatrix3x3 btrot(rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
		btQuaternion btquat;
		btrot.getRotation(btquat);

		btTransform shapeTrans(btquat,btVector3(offset.x(),offset.y(),offset.z()));
		shape->addChildShape(shapeTrans,capsule);
		shape->setMargin(collisionMargin);

		btCollisionObject *collisionObject = new btCollisionObject();
		collisionObject->setCollisionShape(shape);
		if (body)
		{
			collisionObject->setUserPointer(body);
		}
		else
		{
			// Not associated with a body set it's pointer to 0
			collisionObject->setUserPointer(0);
			// No rotation positioned at 0,0,0
			btTransform collisionTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
			collisionObject->setWorldTransform(collisionTransform);
		}
		m_btCollisionWorld->addCollisionObject(collisionObject);
		return;
	}

	void ForceContact::addCylinder(double radius, double width, vector<double> offset, vector<double> rot, BodyRigid *body)
	{
		Vect3 offset_(offset[0],offset[1],offset[2]);
		Mat3x3 rot_;
		rot_ << rot[0], rot[1],	rot[2],
				rot[3],	rot[4],	rot[5],
				rot[6],	rot[7],	rot[8];
		return addCylinder(radius,width,offset_,rot_, body);
	}

	void ForceContact::addCylinder(double radius, double width, Vect3 offset, Mat3x3 rot, BodyRigid * body)
	{
	    radius *= collisionWorldScale;   // shouldn't have to do this but can't get rid of 0.08 contact margin
	    width *= collisionWorldScale;
	    offset *= collisionWorldScale;

		btCylinderShape *cylinder = new btCylinderShape(btVector3(radius,width/2,radius));
		cylinder->setMargin(collisionMargin);  // have to set it on cylinder not on compound shape
		//btCylinderShape *cylinder = new btCylinderShape(btVector3(width,width,radius));
		//btCylinderShape *cylinder = new btCylinderShape(btVector3(radius,width,width));
		btCompoundShape *shape = new btCompoundShape();

		btMatrix3x3 btrot(rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
		btQuaternion btquat;
		btrot.getRotation(btquat);

		btTransform shapeTrans(btquat,btVector3(offset.x(),offset.y(),offset.z()));
		shape->addChildShape(shapeTrans,cylinder);
		shape->setMargin(collisionMargin);

		btCollisionObject *collisionObject = new btCollisionObject();
		collisionObject->setCollisionShape(shape);
		if (body)
		{
			collisionObject->setUserPointer(body);
		}
		else
		{
			// Not associated with a body set it's pointer to 0
			collisionObject->setUserPointer(0);
			// No rotation positioned at 0,0,0
			btTransform collisionTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
			collisionObject->setWorldTransform(collisionTransform);
		}
		m_btCollisionWorld->addCollisionObject(collisionObject);
		return;
	}

	void ForceContact::relocateContactObjects()
	{
	    btCollisionObjectArray &objectArray = m_btCollisionWorld->getCollisionObjectArray();
		// loop through and position collision objects in the collision world
		for (int i=0; i < objectArray.size(); i++)
		{
			btCollisionObject *object = objectArray[i];

//			object->getCollisionShape()->setMargin(collisionMargin);
//			std::cout << (object->getCollisionShape()->getMargin()) << std::endl;
//			std::cout << "relocate in for loop" << std::endl;

			if (object->getUserPointer()) //
			{
				BodyRigid * b = static_cast<BodyRigid *>(object->getUserPointer());
				std::string bodyName = b->m_name;
				Vect3 pos = b->m_pos;
				pos *= collisionWorldScale;   // shouldn't have to do this but can't get rid of 0.08 contact margin

				Mat3x3 rot = b->getRot();
				btVector3 btpos(pos.x(),pos.y(),pos.z());
				btMatrix3x3 btrot(rot(0,0),rot(0,1),rot(0,2),rot(1,0),rot(1,1),rot(1,2),rot(2,0),rot(2,1),rot(2,2));
				btQuaternion btquat;
				btrot.getRotation(btquat);
				btTransform bttrans(btquat,btpos);
				//std::cout << "Body : " << b->GetName() <<", "<< btquat.w() <<", " << btquat.x()<< ", "<< btquat.y() << ", "<<btquat.z() << std::endl;
				object->setWorldTransform(bttrans);
			}
		}
	}

	void ForceContact::storeContactInfo(BodyRigid* b, Vect3 f, Vect3 p)
	{
	    Mat3x3 A = b->getRot();
		Quat q = b->m_q;
	    Vect3 r = b->m_pos;

	    Vect3 posLocal = q.inverse() * (p-r); // in local
	    Vect3 fLocal = q.inverse() *f;

		Vect6 posf;
		posf << posLocal.x(), posLocal.y(), posLocal.z(), fLocal.x(), fLocal.y(), fLocal.z();
		std::vector<double> *t = &(m_c[b]);
		t->push_back(posf(0));
		t->push_back(posf(1));
		t->push_back(posf(2));
		t->push_back(posf(3));
		t->push_back(posf(4));
		t->push_back(posf(5));
	}

	void ForceContact::addTire(double radius, double width, Vect3 offset, Mat3x3 rot, BodyRigid *body,  OMD::Curve2DLinInterp slipCurve)
	{
		btCylinderShape *cylinder = new btCylinderShape(btVector3(radius,width/2,radius));
		cylinder->setMargin(0.001);
		Quat qq(rot);
		addShape(cylinder,offset,qq,body);
      //mTireBodySlipCurveMap.insert(pair<Body *, Curve2DLinInterp> (body,slipCurve));
      mTireBodySlipCurveMap[body] = slipCurve;

		return;
		//mTireBodies.push_back(body);
	}

	void ForceContact::addShape(btCollisionShape* shape, Vect3 offset, Quat const & qt, Body * body)
	{
		// check to see if we already have a btCompoundShape to which we can add the shape
		btCollisionObjectArray &objectArray = m_btCollisionWorld->getCollisionObjectArray();
		btCompoundShape *cShape(0);
		// loop through and position collision objects in the collision world
		for (int i=0; i < objectArray.size(); i++)
		{
			btCollisionObject *object = objectArray[i];
			Body * b = static_cast<Body *>(object->getUserPointer());
			if (body == b)
			{
				cShape = dynamic_cast<btCompoundShape*>(object->getCollisionShape());
		      //btMatrix3x3 btrot(rot[0][0],rot[0][1],rot[0][2],rot[1][0],rot[1][1],rot[1][2],rot[2][0],rot[2][1],rot[2][2]);
		      btQuaternion btquat(qt.x(),qt.y(),qt.z(),qt.w());
		      //btrot.getRotation(btquat);

		      btTransform shapeTrans(btquat,btVector3(offset.x(),offset.y(),offset.z()));
		      cShape->addChildShape(shapeTrans,shape);
				break;
			}
		}
		// if we didn't find a shape already associated with a body make one
		if (!cShape)
		{
			cShape = new btCompoundShape();

		//btMatrix3x3 btrot(rot[0][0],rot[0][1],rot[0][2],rot[1][0],rot[1][1],rot[1][2],rot[2][0],rot[2][1],rot[2][2]);
		//btQuaternion btquat;
		//btrot.getRotation(btquat);
		btQuaternion btquat(qt.x(),qt.y(),qt.z(),qt.w());

		btTransform shapeTrans(btquat,btVector3(offset.x(),offset.y(),offset.z()));
		cShape->addChildShape(shapeTrans,shape);
		cShape->setMargin(0.001);
		//btDefaultMotionState* MotionState = new btDefaultMotionState();  ////////////////////////////////////////////////////////
		
		btCollisionObject *collisionObject = new btCollisionObject();
		collisionObject->setCollisionShape(cShape);
		//btRigidBody *btBody(0);
		if (body)
		{
			//btBody = new btRigidBody(1,MotionState,cShape,btVector3(0,0,0));
			//btBody->setUserPointer(body);
			collisionObject->setUserPointer(body);
		}
		else
		{
			// Not associated with a body set it's pointer to 0
			//btBody = new btRigidBody(0,MotionState,cShape,btVector3(0,0,0));
			collisionObject->setUserPointer(0);
			// No rotation positioned at 0,0,0 
			//btTransform collisionTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
			//collisionObject->setWorldTransform(collisionTransform);
			btTransform collisionTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
			collisionObject->setWorldTransform(collisionTransform);
		}
		m_btCollisionWorld->addCollisionObject(collisionObject);
		//m_btDDynamicsWorld->addRigidBody(btBody);
      }
      return;
	}
}
