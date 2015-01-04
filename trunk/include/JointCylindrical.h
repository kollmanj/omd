#ifndef OMDOMDJOINTCYLINDRICAL_H
#define OMDOMDJOINTCYLINDRICAL_H

#include "JointAxis.h"
namespace OMD
{
	///
	///	@author John Kollman
	///
	class JointCylindrical : public JointAxis
	{
		public:
			JointCylindrical ( string const& name, BodyRigid* parent, Vect3 parent2joint, BodyRigid* child, Vect3 joint2child, Vect3 axis,double q0=0, double u0=0 );
			JointCylindrical ( std::string const & name, BodyRigid* parent, vector<double> parent2joint, BodyRigid* child, vector<double> joint2child, vector<double> axis,double q0=0, double u0=0 );

			~JointCylindrical();

    ///
    /// construct: \f$ ^{k_0}A^k \f$ using an the axis and angle
    ///
    ///
    /// Modified Constraint Jacobian
    /// page 299 Nikravesh
    virtual MatNxN getJacobianModified( int ParentOrChild);
    virtual MatNxN getJacobian( int ParentOrChild );
    virtual VectN getGamaPound();
    virtual vector<double> getViolation();
   private:
      //friend class ForceRevJnt;
      //friend class ForceRevJntPIDCurve2D;
      //friend class ForceRevJntSpringDamp;


};
};
#endif
