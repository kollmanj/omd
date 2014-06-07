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
#ifndef OMD_MODEL3_H
#define OMD_MODEL3_H
#include "Model1.h"
//#include "OMDFwd.h"
#include "BodyRigid.h"
#include "JointRevolute.h"
#include "JointTranslational.h"
#include "JointSpherical.h"
#include "JointCylindrical.h"
#include "JointUniversal.h"
#include "Force1Body.h"
#include "ForceRevJnt.h"
#include "ForceTransJnt.h"
#include "ForceTransJntSpringDamp.h"
#include "ForceRevJntSpringDamp.h"
#include "ForceRevJntPIDCurve2D.h"
#include "ForceGravity.h"
#include "Force2BodySpringDamp.h"
#include "OMD.h"
//#include "OMDTree.h"
//#include "OMDConfig.h"  // includes USE_BULLET or not
//#ifdef USE_BULLET  // only include this if using bullet
//#include "ForceContact.h"
//#endif
//#ifdef USE_OGRE	// include for ogre
//#include "OMDForceCollisionDynWorld.h"
//#endif
//#include "OMDMatrix3.h"
#include "Curve2DSine.h"
#include "States.h"
#include <string>
#include <vector>
#include <map>
//#include "OMDConstraint.h"

namespace OMD
{
	///	Overarching representation of the OMD model

	class Model3 : virtual public Model1
	{
	public:
		///
		/// Construct a Model3
		///
		Model3();

		///
		/// Destruct this Model3
		///
		~Model3();
		Force *getForce(  string forcename );
		void calcIndependentStates();

		MatNxN getConstraintViolation();
		///
		/// add a joint of type: JointTranslational
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// @param[in] axis defines traslational DOF in parent coordinates
		/// return pointer to joint
		///
		JointTranslational* addJointTrans(string const &name, BodyRigid * parent,
			Vect3 parent2joint,
			BodyRigid *child,
			Vect3 axis,
			double q0 = 0., double u0 = 0.);
		///
		/// add a joint of type: JointTranslational without Eigen, for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// @param[in] axis defines traslational DOF in parent coordinates
		/// return pointer to joint
		///
		JointTranslational* addJointTrans(string const &name, BodyRigid * parent,
			vector<double> parent2joint,
			BodyRigid *child,
			vector<double> axis,
			double q0 = 0., double u0 = 0.);
		///
		/// add a joint of type: JointTranslational
		///
		/// @param[in] name name of joint to add
		/// @param[in] parentname parent body name
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] childname child body name
		/// @param[in] axis defines traslational DOF in parent coordinates
		/// return pointer to joint
		///
		JointTranslational* addJointTrans(string const &name, std::string parentname,
			Vect3 parent2joint,
			std::string childname,
			Vect3 axis,
			double q0 = 0., double u0 = 0.);
		///
		/// add a joint of type: JointTranslational
		///
		/// @param[in] name name of joint to add
		/// @param[in] parentname parent body name
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] childname child body name
		/// @param[in] axis defines traslational DOF in parent coordinates
		/// return pointer to joint
		///
		JointTranslational* addJointTrans(string const &name, std::string parentname,
			vector<double> parent2joint,
			std::string childname,
			vector<double> axis,
			double q0 = 0., double u0 = 0.);
		///
		/// add a joint of type: JointRevolute
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// to joint in parent coordinates
		/// @param[in] axis defines Revolute DOF in parent coordinates
		/// return pointer to joint
		///
		JointRevolute* addJointRevolute(	string const& name,
			BodyRigid * parent,
			Vect3 parent2joint,
			BodyRigid* child,
			Vect3 axis,
			double q0 = 0,
			double u0 =0);
		///
		/// add a joint of type: JointRevolute without Eigen, for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// to joint in parent coordinates
		/// @param[in] axis defines Revolute DOF in parent coordinates
		/// return pointer to joint
		///
		JointRevolute* addJointRevolute(	string const& name,
			BodyRigid * parent,
			vector<double> parent2joint,
			BodyRigid* child,
			vector<double> axis,
			double q0 = 0,
			double u0 =0);
		///
		/// add a joint of type: JointRevolute without Eigen, for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// to joint in parent coordinates
		/// @param[in] axis defines Revolute DOF in parent coordinates
		/// return pointer to joint
		///
		JointRevolute* addJointRevolute(	string const& name,
			std::string parentname,
			vector<double> parent2joint,
			std::string childname,
			vector<double> axis,
			double q0 = 0,
			double u0 =0);
		///
		/// add a joint of type: JointCylindrical
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// @param[in] axis defines Revolute DOF in parent coordinates
		/// return pointer to joint
		///
		JointCylindrical* addJointCylindrical(	string const& name,
			BodyRigid * parent,
			Vect3 parent2joint,
			BodyRigid* child,
			Vect3 axis,
			double q0 = 0,
			double u0 =0);
		///
		/// add a joint of type: JointCylindrical without Eigen, for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// @param[in] axis defines Revolute DOF in parent coordinates
		/// return pointer to joint
		///
		JointCylindrical* addJointCylindrical(	string const& name,
			BodyRigid * parent,
			vector<double> parent2joint,
			BodyRigid* child,
			vector<double> axis,
			double q0 = 0,
			double u0 =0);
		///
		/// add a joint of type: JointSpherical
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// return pointer to joint
		///
		JointSpherical* addJointSpherical( string const& name,
			BodyRigid * parent,
			Vect3 parent2joint,
			BodyRigid* child
			);
		///
		/// add a joint of type: JointSpherical without Eigen, for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// return pointer to joint
		///
		JointSpherical* addJointSpherical( string const& name,
			BodyRigid * parent,
			vector<double> parent2joint,
			BodyRigid* child
			);
		///
		/// add a joint of type: addJointUniversal 
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] pAxis axis of revolution in parent coord
		/// @param[in] child child body
		/// @param[in] cAxis axis of revolution in child coord
		/// return pointer to joint
		///
		JointUniversal* addJointUniversal( string const& name,
                                            BodyRigid* parent,
                                            Vect3 parent2joint,
                                            Vect3 pAxis,
                                            BodyRigid* child,
                                            Vect3 cAxis);
		///
		/// add a joint of type: addJointUniversal without Eigen, for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] pAxis axis of revolution in parent coord
		/// @param[in] child child body
		/// @param[in] cAxis axis of revolution in child coord
		/// return pointer to joint
		///
		JointUniversal* addJointUniversal( string const& name,
                                            BodyRigid* parent,
                                            vector<double> parent2joint,
                                            vector<double> pAxis,
                                            BodyRigid* child,
                                            vector<double> cAxis);
		///
		/// add a joint of type: addJointUniversal without Eigen, for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parentname name parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] pAxis axis of revolution in parent coord
		/// @param[in] childname name of child body
		/// @param[in] cAxis axis of revolution in child coord
		/// return pointer to joint
		///
		JointUniversal* addJointUniversal( string const& name,
                                           string const& parentname,
                                            vector<double> parent2joint,
                                            vector<double> pAxis,
                                            string const& childname,
                                            vector<double> cAxis);

		MatNxN getMassMatrix(){return massMatrix;};
		///
		/// Solve for derivatives prior to solving differential equations
		///
		/// @param[in] t time
		/// return Nothing
		///
		virtual std::vector<double> solve(double t, bool storeAccels = false );

		/// 
		/// add a Ridid Body to the model
		/// 
		/// @param[in] name : name of the body
		/// @param[in] mass : mass of the body
		/// @param[in] inertia : inertia of the body
		/// @param[in] pos:  position of the center of the mass of the body
		/// @param[in] q: orientaion of the body in the form of a quaternion
		/// @param[in] vel: initial velocity of the body at the center of mass of the body
		/// @param[in] wl: initial angular velocity of the body
		/// @param[in] fixed:  true if the body is fixed to ground and does not move, otherwise false
		///
		/// @return pointer to the body added to the model
		///
		BodyRigid* addBodyRigid(std::string const &name, 
									double const &mass, 
									Mat3x3 const &inertia, 
									Vect3 const &pos=Vect3(0,0,0), 
									Quat const &q=Quat(1,0,0,0), 
									Vect3 const &vel=Vect3(0,0,0), 
									Vect3 const &wl=Vect3(0,0,0), 
									bool const &fixed=false);
		/// 
		/// add a Ridid Body to the model using no eigen, for use in swig
		/// 
		/// @param[in] name : name of the body
		/// @param[in] mass : mass of the body
		/// @param[in] inertia : inertia of the body
		/// @param[in] pos:  position of the center of the mass of the body
		/// @param[in] q: orientaion of the body in the form of a quaternion
		/// @param[in] vel: initial velocity of the body at the center of mass of the body
		/// @param[in] wl: initial angular velocity of the body
		/// @param[in] fixed:  true if the body is fixed to ground and does not move, otherwise false
		///
		/// @return pointer to the body added to the model
		///
		BodyRigid* addBodyRigid(std::string const &name, double const &mass, std::vector<double> const &inertia, std::vector<double> const &pos, std::vector<double> q, std::vector<double> const &vel, std::vector<double> const &wl, bool const &fixed=false);

		BodyRigid* addBodyRigid (  string name,
			double mass ,
			Mat3x3 inertia,
			Vect3 position ,
			Mat3x3 orientation,
			Vect3 initialVelLocal = Vect3(0,0,0),
			Vect3 initialAngVelLocal = Vect3(0,0,0),
			bool fixed = false);

		ForceRevJnt* addForceRevJnt(string const& name,
			JointRevolute * jnt,
			double trq);

		///
		/// Add Force of type ForceRevJntPIDCurve2D
		/// TODO put this back in
		///
		/// @param[in] name name of force
		/// @param[in] jnt joint on which force is applied
		/// @param[in] p proportional constant
		/// @param[in] i integral constant
		/// @param[in] d damping constant
		/// @param[in] curve the angle in radians the pid will attempt to produce
		/// return a pointer to the curve
		///
		ForceRevJntPIDCurve2D * addForceRevJntPIDCurve2D(string const& name,
			JointRevolute * jnt,
			double p,
			double i,
			double d,
			Curve2DSine *curve);

		/// TODO: put this back in
		//ForceRevJntPIDCurve2D * addForceRevJntPIDCurve2D(string const& name,
		//	JointRevolute * jnt,
		//	double p,
		//	double i,
		//	double d,
		//	std::vector<Curve2D *> curves);

		ForceRevJntSpringDamp * addForceRevJntSpringDamp(string const& name,
			JointRevolute *jnt,
			double k,
			double c,
			double fl=0);

		Vect3 getBodyPosition( string bodyname );
		Mat3x3 getBodyRot( string bodyname );
		MatNxN getJacobianModified();
		MatNxN getJacobian( );
		VectN getGamaP();

		/// TODO maybe move this to model1
		int m_fixedRigidBodyCount;

//#ifdef USE_BULLET
//		ForceContact * addForceContact ( string const& name, btCollisionObjectArray objects, double stiff, double damp, double frict, double thresh  );
//		ForceContact * addForceContact ( string const& name, btCollisionWorld *collisionWorld, double stiff, double damp, double frict, double thresh);
//		ForceContact * addForceContact ( string const& name, double stiff, double damp, double frict, double thresh);
//#endif
//#ifdef USE_OGRE	// don't include in SWIG
//		ForceCollisionDynWorld * addForceCollisionDynWorld ( string const& name, double stiff, double damp, double frict, double thresh );
//#endif
		//ForceGravity * addForceGravity ( string const& name, Vect3 g);

		virtual std::vector<double> getState( );
		virtual std::vector<double> getDot( );
		//		std::vector<BodyRigid *>* getBodies(){return &m_bodies;};
		virtual void setState( std::vector<double> state_vector );
		virtual unsigned int getStateSize();
		MatNxN getForceAndTorqueVect(){return forceAndTorque;};

		//Force1Body * addForce1Body ( string const& name,
		//	BodyRigid *body,
		//	Vect3 force,
		//	Vect3 torque = Vect3(0.,0.,0.),
		//	Vect3 coord = Vect3(0.,0.,0.) );

	private:
		//			vector<Tree *> m_trees;
		//			vector<Branch *> m_branches;

		//		std::map<string, Force *> m_forces;
		std::vector<double> state_vector;
//		std::vector<double> state_vector_dot;
		std::vector<Joint *> m_joint;
		//		std::vector<Constraint *> m_constraint;
		MatNxN massMatrix;
		/// Constraint jacobian
//		MatNxN jacobian;
		/// Modified Jacobian
		//        MatNxN  jacobianM;
		//        MatNxN gamaP;

		/// Force and Torque vector
		MatNxN forceAndTorque;
		MatNxN getAllAccels();
		MatNxN NewtonRaphson(double t);
		void stepC3( );

//		void calcIndependentStateIndices(double t);
//		std::vector<int> m_IndependentIndices;
//		std::vector<int> m_DependentIndices;
		// time
		double m_time0;
		double m_time1;
		States m_states;
		double m_epsilon;
		std::vector<int> jacobianRows2Drop;

	};
};
#endif
