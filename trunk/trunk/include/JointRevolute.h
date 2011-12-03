#ifndef OMDJOINTREVOLUTE_H
#define OMDJOINTREVOLUTE_H

#include "Joint1dof.h"
namespace OMD
{
	///
	///	@author John Kollman
	///
	class JointRevolute : public Joint1DOF
	{
		public:
	///
	/// Construct a JointRevolute joint
	///
	/// @param[in] name	name of joint
	/// @param[in] parent parent body
	/// @param[in] parent2joint is vector from parent cg to joint
	/// @param[in] child child body
	/// @param[in] joint2child is the vector from joint to childs
	/// @param[in] axis direction DOF in parent
	/// @return The status of the call
	///
	JointRevolute ( string const& name, BodyRigid* parent, Vect3 parent2joint, BodyRigid* child, Vect3 joint2child, Vect3 axis,double q0=0, double u0=0 );

	///
	/// Construct a JointRevolute joint
	///
	/// @param[in] name	name of joint
	/// @param[in] parent parent body
	/// @param[in] parent2joint is vector from parent cg to joint
	/// @param[in] child child body
	/// @param[in] joint2child is the vector from joint to childs
	/// @param[in] axis direction DOF in parent
	/// @return The status of the call
	///
	JointRevolute ( string const& name, BodyRigid* parent, vector<double> parent2joint, BodyRigid* child, vector<double> joint2child, vector<double> axis,double q0=0, double u0=0 );

	~JointRevolute();

    ///
    /// construct: \f$ ^{k_0}A^k \f$ using an the axis and angle
    ///
    virtual void kinematicsPosition( double t );
    virtual Vect3 calcChildVel( double t );
    ///
    /// Equation 37 & 38 [Anderson  p51] for revolute joint \n
    /// returns \f$ ^{N} \omega ^k \f$
    virtual Vect3 calcChildAngVel( double t );
	///
	/// Equation 46 page 52 Anderson
    virtual Vect3 KinematicsAccelLin( double t );
	///
	/// Equation 45 page 52 Anderson
    virtual Vect3 KinematicsAccelAng( double t );

    ///
    /// Modified Constraint Jacobian
    /// page 299 Nikravesh
    virtual MatNxN getJacobianModified( int ParentOrChild);
    virtual MatNxN getJacobian( int ParentOrChild );
    virtual VectN getGamaPound();
    virtual vector<double> getViolation();
	///
	/// sorry for the terrible name but it's physical significance is ?
	/// this is equestion 41 page 51 of Anderson
	///
	Vect3 Nv_r();
	/// this is equestion 40 page 51 of Anderson
	Vect3 Nw_r();
   private:
      friend class ForceRevJnt;
      friend class ForceRevJntPIDCurve2D;
      friend class ForceRevJntSpringDamp;


	};

};
#endif