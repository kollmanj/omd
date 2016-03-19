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
#ifndef OMDFORCECONTACT_H
#define OMDFORCECONTACT_H

#include "Force.h"
//#include "OMD.h"
#include "BodyRigid.h"
#include <BulletCollision/BroadphaseCollision/btAxisSweep3.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <vector>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <LinearMath/btAlignedObjectArray.h>
#include "Curve2DLinInterp.h"
#include <map>
//#include "OMDCurve2DLinInterp.h"

namespace OMD
{
//using namespace OMD;
	/**
		@author John Kollman,,, <john@john>
	*/
	class ForceContact : public Force
	{
		public:
			ForceContact ( string const& name, btCollisionObjectArray objects, double stiff, double damp, double frict, double thresh );
			ForceContact ( string const& name, btCollisionWorld *collisionWorld, double stiff, double damp, double frict, double thresh );
			ForceContact ( string const& name, double stiff, double damp, double frict, double thresh );
			///
			/// add a Box shape to the body which will collide with other contact geometry in the model
			///
			/// @param[in] x
			/// @param[in] y
			/// @param[in] z
			/// @param[in] offset, distance of the center of the box from the cg of the body
			/// @param[in[ rot, rotation describing the orienation of the box relative to that of the body
			///
			/// @return nothing
			///
			void addBox(double x, double y, double z, Vect3 offset = Vect3(0,0,0), Mat3x3 rot = Mat3x3::Identity(),BodyRigid *body=0);
			void addBox(double x, double y, double z, std::vector<double> offset, std::vector<double> rot, BodyRigid *body=0);
			void addBox(double x, double y, double z, Vect3 offset = Vect3(0,0,0), Quat rot = Quat(1,0,0,0),BodyRigid *body=0);
			void addCapsule(double radius, double height, Vect3 offset = Vect3(0,0,0), Mat3x3 rot = Mat3x3::Identity(),BodyRigid *body=0);
			void addCapsule(double radius, double height, std::vector<double> offset, std::vector<double> rot, BodyRigid *body=0);
			void addSphere(double radius, Vect3 offset = Vect3(0,0,0), Mat3x3 rot = Mat3x3::Identity(),BodyRigid *body=0);
			void addSphere(double radius, std::vector<double> offset, std::vector<double> rot, BodyRigid *body=0);
			void addCylinder(double radius, double width, Vect3 offset= Vect3(0,0,0), Mat3x3 rot= Mat3x3::Identity(), BodyRigid * body=0);
			void addCylinder(double radius, double width, std::vector<double> offset, std::vector<double> rot, BodyRigid *body=0);
			void addCylinder(double radius, double width, Vect3 const & offset, Vect3 const & axis, Vect3 const & fwd, BodyRigid * body);
			void addTire(BodyRigid *body);
			void addTire(double radius, double width, Vect3 offset, Mat3x3 rot, BodyRigid * body, OMD::Curve2DLinInterp slipCurve);
			std::vector<double> getContactForces(Body *b){return m_c[b];};
        // void assignFrictionCurve(Curve2DLinInterp curve);

			~ForceContact();

			virtual void apply(double t);

			btCollisionWorld* getbtCollisionWorld();
			btCollisionConfiguration* GetbtCollisionConfiguration(){return mCollisionConfig;};
			void setCollisionMargin(double cm){collisionMargin=cm;};
			void setCollisionWorldScale(double s){collisionWorldScale=s;};
			//double getCollisionMargin(Body *b);
			void addShape(btCollisionShape* shape, Vect3 offset = Vect3(0,0,0), Quat const &qt= Quat(1,0,0,0), BodyRigid * body=0);


		private:
//            void calcNormalForceAndTorque(Body *b, Vect3 ptLocal, double dist, Vect3 normG, Vect3 &f, Vect3 &t);
//            void calcFrictionForceAndTorque(Body *b, Vect3 ptLocal,Vect3 ptRelativeVelGlobal, Vect3 normalF, Vect3 &f, Vect3 &t);
			btAxisSweep3 *mBroadphase;
			//btCollisionObjectArray m_collisionObjects;
			btDefaultCollisionConfiguration *mCollisionConfig;
			btCollisionDispatcher *mDispatcher;
			btCollisionWorld *m_btCollisionWorld;
			std::vector<BodyRigid *> mTireBodies;
			double mk, md, m_mu, m_thresh;
			// stores information about contact
			std::map<Body*, std::vector<double> > m_c;
			void relocateContactObjects();
			void storeContactInfo(BodyRigid* b, Vect3 f, Vect3 p);
			double collisionMargin;
			double collisionWorldScale;  // ugh, I shouldn't need this but 0 margin results in 0.08 "margin" !
			std::map<Body *, Curve2DLinInterp> mTireBodySlipCurveMap;
	};

}

#endif
