#ifndef OMD_MODEL2_H
#define OMD_MODEL2_H

#include "Model1.h"
#include <vector>
#include <map>
#include <iostream>
#include <string>
#include "OMD.h"
#include "JointTranslational.h"
#include "Tree.h"
#include "JointRevolute.h"
#include "ForceRevJntPIDCurve2D.h"
#include "Curve2DSine.h"
#include "ForceRevJntSpringDamp.h"

//#ifdef USE_BULLET
//#include "ForceContact.h"
//#endif:

namespace OMD
{
	///	Overarching representation of the OMD model

	class Model2 : public Model1
	{
	public:
		///
		/// Construct a Model1
		///
		Model2();

		///
		/// Destruct this Model2
		///
		~Model2();
		///
		/// add a joint of type: JointTrans
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// @param[in] joint2child vector describing location of child relative
		/// to joint in parent coordinates
		/// @param[in] axis defines traslational DOF in parent coordinates
		/// return pointer to joint
		///
		JointTranslational* addJointTranslational(string const &name, BodyRigid * parent,
			Vect3 parent2joint,
			BodyRigid *child,
			Vect3 joint2child,
			Vect3 axis,
			double q0 = 0., double u0 = 0.);

		///
		/// add a joint of type: JointTrans without Eigen for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// @param[in] joint2child vector describing location of child relative
		/// to joint in parent coordinates
		/// @param[in] axis defines traslational DOF in parent coordinates
		/// return pointer to joint
		///
		JointTranslational* addJointTranslational(string const &name, BodyRigid * parent,
			std::vector<double> const &parent2joint,
			BodyRigid *child,
			std::vector<double> const &joint2child,
			std::vector<double> const & axis,
			double q0 = 0., double u0 = 0.);

		///
		/// add a joint of type: JointTrans
		///
		/// @param[in] name name of joint to add
		/// @param[in] parentname parent body name
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] childname child body name
		/// @param[in] joint2child vector describing location of child relative
		/// to joint in parent coordinates
		/// @param[in] axis defines traslational DOF in parent coordinates
		/// return pointer to joint
		///
		JointTranslational* addJointTranslational(string const &name, std::string parentname,
			Vect3 parent2joint,
			std::string childname,
			Vect3 joint2child,
			Vect3 axis,
			double q0 = 0., double u0 = 0.);

		///
		/// add a joint of type: JointTrans without Eigen for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parentname parent body name
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] childname child body name
		/// @param[in] joint2child vector describing location of child relative
		/// to joint in parent coordinates
		/// @param[in] axis defines traslational DOF in parent coordinates
		/// return pointer to joint
		///
		JointTranslational* addJointTranslational(string const &name, std::string parentname,
			std::vector<double> parent2joint,
			std::string childname,
			std::vector<double> joint2child,
			std::vector<double> axis,
			double q0 = 0., double u0 = 0.);

		///
		/// add a joint of type: JointRevolute without Eigen for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// @param[in] joint2child vector describing location of child relative
		/// to joint in parent coordinates
		/// @param[in] axis defines Revolute DOF in parent coordinates
		/// return pointer to joint
		///
		JointRevolute* addJointRevolute(	string const& name,
			BodyRigid * parent,
			std::vector<double> parent2joint,
			BodyRigid* child,
			std::vector<double> joint2child,
			std::vector<double> axis,
			double q0 = 0,
			double u0 =0);

		///
		/// add a joint of type: JointRevolute
		///
		/// @param[in] name name of joint to add
		/// @param[in] parent parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] child child body
		/// @param[in] joint2child vector describing location of child relative
		/// to joint in parent coordinates
		/// @param[in] axis defines Revolute DOF in parent coordinates
		/// return pointer to joint
		///
		JointRevolute* addJointRevolute(	string const& name,
			BodyRigid * parent,
			Vect3 parent2joint,
			BodyRigid* child,
			Vect3 joint2child,
			Vect3 axis,
			double q0 = 0,
			double u0 =0);

		///
		/// add a joint of type: JointRevolute without Eigen for use with swig
		///
		/// @param[in] name name of joint to add
		/// @param[in] parentname name of parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] childname name of child body
		/// @param[in] joint2child vector describing location of child relative
		/// to joint in parent coordinates
		/// @param[in] axis defines Revolute DOF in parent coordinates
		/// return pointer to joint
		///
		JointRevolute* addJointRevolute(	string const& name,
			std::string parentname,
			std::vector<double> parent2joint,
			std::string childname,
			std::vector<double> joint2child,
			std::vector<double> axis,
			double q0 = 0,
			double u0 =0);

		///
		/// add a joint of type: JointRevolute
		///
		/// @param[in] name name of joint to add
		/// @param[in] parentname name of parent body
		/// @param[in] parent2joint vector describing location of joint relative
		/// to parent in parent coordinates
		/// @param[in] childname name of child body
		/// @param[in] joint2child vector describing location of child relative
		/// to joint in parent coordinates
		/// @param[in] axis defines Revolute DOF in parent coordinates
		/// return pointer to joint
		///
		JointRevolute* addJointRevolute(	string const& name,
			std::string parentname,
			Vect3 parent2joint,
			std::string childname,
			Vect3 joint2child,
			Vect3 axis,
			double q0 = 0,
			double u0 =0);


		Joint *getJoint( 	string jointname );
		Force *getForce(  string forcename );

		///
		/// Build Tree
		/// Needs to be done after all joints have been added and before model is solved
		/// @return true if successful
		///
		bool buildTree();
		///
		/// Get a tree from the model
		///
		/// @return The tree s
		///
		Tree *getTree();

		///
		/// Solve for derivatives prior to solving differential equations
		///
		/// @param[in] t time
		/// return Nothing
		///
		virtual std::vector<double> solve(double t, bool storeAccels=false );
		///
		/// add a Rigid Body with no eigen for use in swig
		///
		/// @param name name of the body
		/// @param mass mass of the body
		/// @param inertia inertia of the body
		/// @param orientaiton orientation of the body in the form a rotation matrix
		/// @param fixed true if body is fixed to the ground
		/// return rigid body
		///
		BodyRigid* addBodyRigid (  string name,
			double mass,
			std::vector<double> inertia,
			std::vector<double> orientation,
			bool fixed = false);
		///
		/// add a Rigid Body
		///
		/// @param name name of the body
		/// @param mass mass of the body
		/// @param inertia inertia of the body
		/// @param orientaiton orientation of the body in the form a rotation matrix
		/// @param fixed true if body is fixed to the ground
		/// return rigid body
		///
		BodyRigid* addBodyRigid (  string name,
			double mass,
			Mat3x3 inertia,
			Mat3x3 orientation,
			bool fixed = false);

		Vect3 getBodyPosition( string bodyname );
		Mat3x3 getBodyRot( string bodyname );

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
		// TODO: put this back in
		//Curve2DSineWave * addCurve2DSineWave(string const& name, double amp, double offset, double freq, double shift);

		/// this now exists in Model1 the parent of Model2
		//Force1Body * addForce1Body ( string const& name,
		//	Body *body,
		//	Vect3 force,
		//	Vect3 torque = Vect3(0.,0.,0.),
		//	Vect3 coord = Vect3(0.,0.,0.) );
// inherited from Model1
//#ifdef USE_BULLET
//		ForceContact * addForceContact ( string const& name, btCollisionObjectArray objects, double stiff, double damp, double frict, double thresh  );
//		ForceContact * addForceContact ( string const& name, btCollisionWorld *collisionWorld, double stiff, double damp, double frict, double thresh);
//		ForceContact * addForceContact ( string const& name, double stiff, double damp, double frict, double thresh);
//#endif
//#ifdef USE_OGRE	// don't include in SWIG
//		ForceCollisionDynWorld * addForceCollisionDynWorld ( string const& name, double stiff, double damp, double frict, double thresh );
//#endif
		// inherited from Model1
		//ForceGravity * addForceGravity ( string const& name, Vect3 g);
		//Force2BodySpringDamp * addForce2BodySpringDamp ( string const& name,
		//	Body *body1,
		//	Body *body2,
		//	double k,
		//	double c,
		//	double fl = 0,
		//	Vect3 body1Offset =Vect3(0.,0.,0.),
		//	Vect3 body2Offset =Vect3(0.,0.,0.));
		// ihherited from Model1
		//Force2BodySpringDamp * addForce2BodySpringDamp ( string const& name,
		//	string const& body1Name,
		//	string const& body2Name,
		//	double k,
		//	double c,
		//	double fl = 0,
		//	Vect3 body1Offset =Vect3(0.,0.,0.),
		//	Vect3 body2Offset =Vect3(0.,0.,0.));

		virtual std::vector<double> getState( );
		virtual std::vector<double> getDot( );

		virtual void setState( std::vector<double> state_vector );
		virtual unsigned int getStateSize();
		void addTree( Tree tree );
	private:
		///
		/// Do the kinematics
		///
		/// @param[in]	t time
		/// @return Nothing
		///
		void kinematics(double t);
		///
		/// apply Forces to Body
		///
		/// @param[in]	t time
		/// @return Nothing
		///
		//void applyForces(double t );
		///
		/// Do Dynamics
		///
		/// @param[in]	t time
		/// @return Nothing
		///
		void Dynamics( double t );

		///
		/// key: Joint Value: Child Body
		std::map<Joint1DOF *, BodyRigid *> m_child_map;
		/// key: body parent and value: joint
		std::multimap<BodyRigid *, Joint1DOF *> m_parent_map;
		Tree m_tree;
		//			vector<Tree *> m_trees;
		//			vector<Branch *> m_branches;

		//		std::vector<Body *> m_bodies;
		//TODO: put this back in, but in Model1
		//std::vector<Curve2D *> m_curve2Ds;

	};
};
#endif
