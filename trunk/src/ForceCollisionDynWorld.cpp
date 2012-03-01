#ifndef SWIG	// don't include in SWIG
#include "ForceCollisionDynWorld.h"
//#include "OMDMatrix3.h"

#include <iostream>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
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
	}

	ForceCollisionDynWorld::~ForceCollisionDynWorld()
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
		//m_btCollisionWorld->performDiscreteCollisionDetection(); virtual!
		btCollisionObjectArray &objectArray = m_btDDynamicsWorld->getCollisionObjectArray();
		// loop through and position collision objects in the collision world
		for (int i=0; i < objectArray.size(); i++)
		{
			btCollisionObject *object = objectArray[i];
			if (object->getUserPointer()) //
			{
				BodyRigid * b = static_cast<BodyRigid *>(object->getUserPointer());
				std::string bodyName = b->m_name;
				Vect3 pos = b->m_pos;
				//Mat3x3 rot = b->getRot();
				Quat OMDq = b->m_q;
				btVector3 btpos(pos.x(),pos.y(),pos.z());
				//btMatrix3x3 btrot(rot[0][0],rot[0][1],rot[0][2],rot[1][0],rot[1][1],rot[1][2],rot[2][0],rot[2][1],rot[2][2]);
				btQuaternion btquat(OMDq.x(),OMDq.y(),OMDq.z(),OMDq.w());
				//btrot.getRotation(btquat);
				btTransform bttrans(btquat,btpos);
				//std::cout << "Body : " << b->GetName() <<", "<< btquat.w() <<", " << btquat.x()<< ", "<< btquat.y() << ", "<<btquat.z() << std::endl;
				object->setWorldTransform(bttrans);
			}
			else
			{
				btTransform t = object->getWorldTransform();
				btQuaternion q = t.getRotation();
				//std::cout << "Quat: " << q.w() <<", " << q.x() << ", " << q.y() <<", " << q.z()<< ", "<< std::endl;
			}
		}
		//int numCollisionObjects = m_btCollisionWorld->getNumCollisionObjects();

		m_btDDynamicsWorld->performDiscreteCollisionDetection();
		int numManifolds = m_btDDynamicsWorld->getDispatcher()->getNumManifolds();


		//        std::cout << "number of manifolds: " << numManifolds << std::endl;
		//        std::cout << "number of collision objects: " << numCollisionObjects << std::endl;
		for (int i=0;i<numManifolds;i++)
		{
			btPersistentManifold* contactManifold = m_btDDynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
			btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
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
         else
         {
            AisGrnd = true;
            //std::cout << "A is Ground!!!" << std::endl;
         }

         if ( bB )
         {
            if(bB->isFixed())
            {
               BisGrnd = true;
               //std::cout << "B is Ground (cause it is fixed)!!!" << std::endl;
            }
         }
         else
         {
            BisGrnd = true;
               //std::cout << "B is Ground!!!" << std::endl;
         }

         bool tireGrndInteraction = false;
         if ( (AisTire && BisGrnd) || (BisTire && AisGrnd) )
         {
            tireGrndInteraction = true;
         }

		 bool nontirenongrndInteraction = false;
		 if (( !AisTire && !BisTire) )
		 {
			 nontirenongrndInteraction = true;
		 }


         //mTireBodySlipCurveMap[bB];

			/// TIRE STUFF BEGIN ///
			// see if body A is a Tire
			//bool bAisTire = false;
			//vector<OMD::Body *>::iterator it;
			//for ( it=mTireBodies.begin() ; it < mTireBodies.end(); it++ )
			//{
			//	if (bA==(*it) && (!bB || bB->IsFixed()) )
			//	{	
			//		bAisTire = true;
			//		break;
			//	}
			//}
			//// see if body B is a Tire
			//bool bBisTire = false;
			////vector<OMD::Body *>::iterator it;
			//for ( it=mTireBodies.begin() ; it < mTireBodies.end(); it++ )
			//{
			//	if (bB==(*it) && (!bA || bA->IsFixed()) )
			//	{	
			//		bAisTire = true;
			//		break;
			//	}
			//}
			/// TIRE STUFF END ///


			int numContacts = contactManifold->getNumContacts();
			// velocity of the contact point on body A, fill with 0 in case not associated with a body
			Vect3 cntctPntBAVelGlobal(0,0,0);
			// velocity of the contact point on body B, fill with 0 in case not associated with a body
			Vect3 cntctPntBBVelGlobal(0,0,0);

			if (bA)
			{
//            std::cout << "body A: " << bA->GetName() << std::endl;
				for (int j=0;j<numContacts;j++)
				{
					btManifoldPoint& pt = contactManifold->getContactPoint(j);
					btScalar distance = pt.getDistance();
					if (distance<0.f)
					{
						Vect3 fn,tn,ff,tf;
						const btVector3& ptA = pt.m_localPointA;
						const btVector3& normalOnB = pt.m_normalWorldOnB;
                  // reduce ptLocal by distance
                  OMD::Vect3 ptALocal(ptA.x(),ptA.y(),ptA.z());
				  OMD::Vect3 temp = ptALocal.normalized();
                  //temp.normalize();
                  ptALocal = ptALocal + temp*distance;
						calcNormalForceAndTorque(bA,ptALocal,distance,Vect3(normalOnB.x(),normalOnB.y(),normalOnB.z()),fn,tn);
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
						//std::cout << "Body : " << bA->GetName() <<", "<< ptA.x() <<", " << ptA.y() << ", "<<ptA.z() << std::endl;
						//std::cout << "Body : " << bA->GetName() <<", "<< cntctPntBAVelGlobal.x <<", " << cntctPntBAVelGlobal.y << ", "<<cntctPntBAVelGlobal.z << std::endl;
                  //
                  //
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
						//ff = Vector3(0,0,0);
						//tf = Vector3(0,0,0);
						//bA->ForceAccum(fn+ff,tn+tf);
							 if (tireGrndInteraction || nontirenongrndInteraction)
		 {
						bA->forceAndTorqueAccum(fn+ff,true,Vect3(0,0,0),tn+tf,true);
							 }

						//bB->forceAccumGlobal(fn+ff,ptALocal);
						//double j = fn.dotProduct(ff);
						//std::cout << "Body A: " << bA->GetName() <<" dot: "<<j<< std::endl; 
						//std::cout << "Body A: "<< bA->GetName() << ff.x <<", " << ff.y << ", "<<ff .z << std::endl;
					}
				}
			}
			if (bB)
			{
//            std::cout << "body B: " << bB->GetName() << std::endl;
				for (int j=0;j<numContacts;j++)
				{
					btManifoldPoint& pt = contactManifold->getContactPoint(j);
					btScalar distance = pt.getDistance();
					if (distance<0.f)
					{
						//const btVector3& ptBWorld = pt.getPositionWorldOnB();
						Vect3 fn,tn,ff,tf;
						const btVector3& ptB = pt.m_localPointB;
						const btVector3& normalOnB = pt.m_normalWorldOnB;
                  // reduce ptLocal by distance
                  OMD::Vect3 ptBLocal(ptB.x(),ptB.y(),ptB.z());
                  OMD::Vect3 temp = ptBLocal.normalized();
                  //temp.normalize();
                  ptBLocal = ptBLocal + temp*distance;
						calcNormalForceAndTorque(bB,ptBLocal,distance,Vect3(-normalOnB.x(),-normalOnB.y(),-normalOnB.z()),fn,tn);

						cntctPntBBVelGlobal = bB->getVelocityGlobal(ptBLocal);
						if (bA)
						{
							const btVector3& ptA = pt.m_localPointA;
                  // reduce ptLocal by distance
                  OMD::Vect3 ptALocal(ptA.x(),ptA.y(),ptA.z());
                  OMD::Vect3 temp = ptALocal;
                  temp.normalize();
                  ptALocal = ptALocal + temp*distance;
							cntctPntBAVelGlobal = bA->getVelocityGlobal(ptALocal);
						}
						else
						{
							cntctPntBAVelGlobal = Vect3(0,0,0);
						}

         if (tireGrndInteraction)
         {
                  //doTireFrictionForceAndTorque(Body *b, Vector3 ptLocal,Vector3 ptRelativeVelGlobal,Vector3 normalF, Vector3 &f, Vector3 &t);
						doTireFrictionForceAndTorque(bB,ptBLocal,(cntctPntBBVelGlobal-cntctPntBAVelGlobal), fn, ff, tf);
         }
         else
         {
						calcFrictionForceAndTorque(bB, ptBLocal,(cntctPntBBVelGlobal-cntctPntBAVelGlobal), fn, ff, tf);
         }
						//ff = Vector3(0,0,0);
						//tf = Vector3(0,0,0);
						//bB->ForceAccum(fn+ff,tn+tf);

		 if (tireGrndInteraction || nontirenongrndInteraction)
		 {
						bB->forceAndTorqueAccum(-(fn+ff),true,Vect3(0,0,0),-(tn+tf),true);
		 }
						//bB->forceAccumGlobal(-fn-ff,ptBLocal);
						//double j = fn.dotProduct(ff);
						//std::cout << "Body B: " << bB->GetName() <<" dot: "<<j<< std::endl; 
						//std::cout << "Body B: " << ff.x <<", " << ", "<< ff.y << ", "<<ff .z << std::endl;
					}
				}
			}

		}
		//		Matrix3 RotFromWorld2Body = m_body.GetRot().Transpose();
		//		Vector3 Force_in_local = RotFromWorld2Body * m_force;
		//		Force_in_local = m_force;
		//		Vector3 Torque_in_local = m_torque + m_localcoord.crossProduct(Force_in_local);
		//		m_body.ForceAccum(Force_in_local,Torque_in_local);
		return;
	}

	void ForceCollisionDynWorld::calcNormalForceAndTorque(BodyRigid *b, Vect3 ptLocal, double dist, Vect3 normG, Vect3 &f, Vect3 &t)
	{
		//Mat3x3 BodyRot = b->getRot();
		Quat BodyQuat = b->m_q;

		Vect3 PtVelGlobal = b->getVelocityGlobal(ptLocal);
		//double PtVelInNorm = PtVelGlobal.dotProduct(normG);
		double PtVelInNorm = PtVelGlobal.dot(normG);
		double damp = md*PtVelInNorm;
      Vect3 dampVG(0,0,0);
      if (damp < 0) 
      {
		   dampVG = normG*-damp;
      }
		double stiff = mk*dist;
		Vect3 stiffVG = normG*-stiff;
		// total force in global
		Vect3 fTotalVG = dampVG+ stiffVG;
		//		Vector3 fTotalVG = stiffVG;
		//f = BodyRot.getTranspose()*fTotalVG;
		f = BodyQuat.inverse()*fTotalVG;
		t = ptLocal.cross(f);
		//t = ptLocal.crossProduct(f);
		//        t = Vector3(0,0,0);
      OMD::Vect3 fg =normG;
      //std::cout << "Body: " << b->GetName() << std::endl;
//      std::cout << "x: " << fTotalVG.x << "y: " << fTotalVG.y << "z: " << fTotalVG.z << std::endl;
//      std::cout << "nx: " << normG.x << "ny: " << normG.y << "nz: " << normG.z << std::endl;
      //std::cout << "dampVGx: " << dampVG.x << "dampVGy: " << dampVG.y << "dampVGz: " << dampVG.z << std::endl;
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
		btCollisionObjectArray &objectArray = m_btDDynamicsWorld->getCollisionObjectArray();
		btCompoundShape *cShape(0);
		// loop through and position collision objects in the collision world
		for (int i=0; i < objectArray.size(); i++)
		{
			btCollisionObject *object = objectArray[i];
			BodyRigid * b = static_cast<BodyRigid *>(object->getUserPointer());
			if (body == b)
			{
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
		{
			cShape = new btCompoundShape();

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
      }
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
