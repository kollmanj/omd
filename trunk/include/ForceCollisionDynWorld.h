#ifndef SWIG	// don't include in SWIG
#pragma once
#include "Force.h"
//#include "OMDFwd.h"
#include "OMD.h"
#include "BodyRigid.h"
//#include "Vector3.h"
//#include "Matrix3.h"
#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <vector>
#include <map>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btAlignedObjectArray.h>
#include "Curve2DLinInterp.h"
#include <btBulletDynamicsCommon.h>
#include <fstream>	// write to file for debugging

namespace OMD
{
class ForceCollisionDynWorld :
	public Force
{
		public:
			ForceCollisionDynWorld ( string const& name, double stiff, double damp, double frict, double thresh );
			void addBox(double x, double y, double z, Vect3 offset = Vect3(0,0,0), Mat3x3 rot = Mat3x3(3,3),BodyRigid *body=0);
			void addCapsule(double radius, double height, Vect3 offset = Vect3(0,0,0), Mat3x3 rot = Mat3x3(3,3),BodyRigid *body=0);
			void addCylinder(double radius, double width, Vect3 offset= Vect3(0,0,0), Mat3x3 rot= Mat3x3(3,3), BodyRigid * body=0);
			void addTire(double radius, double width, Vect3 offset, Mat3x3 rot, BodyRigid * body, OMD::Curve2DLinInterp slipCurve);
			void addCompoundShape(btCompoundShape *shape, BodyRigid *body=0);
			//void addTire(Body *body);
			~ForceCollisionDynWorld();
			btDiscreteDynamicsWorld * getbtDiscreteDynamicsWorld(){ return m_btDDynamicsWorld; };
			virtual void apply(double t);

		private:
			void addShape(btCollisionShape* shape, Vect3 offset = Vect3(0,0,0), Mat3x3 rot= Mat3x3(3,3), BodyRigid * body=0);
            void calcNormalForceAndTorque(BodyRigid *b, Vect3 ptLocal, double dist, Vect3 normG, Vect3 &f, Vect3 &t);
            void calcFrictionForceAndTorque(BodyRigid *b, Vect3 ptLocal,Vect3 ptRelativeVelGlobal, Vect3 normalF, Vect3 &f, Vect3 &t);
            void doTireFrictionForceAndTorque(BodyRigid *b, Vect3 ptLocal,Vect3 ptRelativeVelGlobal,Vect3 normalF, Vect3 &f, Vect3 &t);
			btAxisSweep3 *mBroadphase;
			//btCollisionObjectArray m_collisionObjects;
			btDefaultCollisionConfiguration *mCollisionConfig;
			btCollisionDispatcher *mDispatcher;
			btSequentialImpulseConstraintSolver *mSolver;
			btDiscreteDynamicsWorld *m_btDDynamicsWorld;
			std::map<Body *, Curve2DLinInterp> mTireBodySlipCurveMap;
			//std::vector<Body *> mTireBodies;
			//btAlignedObjectArray<btCollisionShape*> mCollisionShapes;
			double mk, md, m_mu, m_thresh;
			ofstream fout;
};
}
#endif
