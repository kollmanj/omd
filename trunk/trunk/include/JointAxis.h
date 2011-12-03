#ifndef OMDJOINTAXIS_H
#define OMDJOINTAXIS_H

#include "Joint.h"

namespace OMD
{
class JointAxis :
	public Joint
{
public:

		/// constructor if the axis and the parent is
		/// same as axis in child
		JointAxis(string const &name,
				BodyRigid *parent,
				vector<double> const &parent2joint,
				BodyRigid *child,
				vector<double> const &joint2child,
				vector<double> const &axisInParent);

		/// constructor if the axis and the parent is
		/// same as axis in child
		JointAxis(string const &name,
				BodyRigid *parent,
				Vect3 parent2joint,
				BodyRigid *child,
				Vect3 joint2child,
				Vect3 axisInParent);
/// Generic Constructor
/// Construct a Model
/// @return Nothing
///
			JointAxis(string const &name,
				BodyRigid *parent,
				Vect3 parent2joint,
				Vect3 axisInParent,
				BodyRigid *child,
				Vect3 joint2child,
				Vect3 axisInChild);

			/// Generic Constructor
/// Construct a Model
/// @return Nothing
///
			JointAxis(string const &name,
				BodyRigid *parent,
				vector<double> const &parent2joint,
				vector<double> const &axisInParent,
				BodyRigid *child,
				vector<double> const &joint2child,
				vector<double> const &axisInChild);


            virtual MatNxN getJacobianModified( int ParentOrChild)=0;
            virtual VectN getGamaPound()=0;
            ///
            /// page 201 table 7.1 Nikravesh
            ///
            MatNxN getC(int ParentOrChild = PARENT);
            MatNxN getC2(int ParentOrChild = PARENT);
			virtual ~JointAxis(){};
			virtual vector<double> getViolation()=0;
			/////
			///// sorry for the terrible name but it's physical significance is ?
			///// this is equestion 41 page 51 of Anderson
			/////
			//virtual Vect3 Nv_r()=0;
			///// this is equestion 40 page 51 of Anderson
			//virtual Vect3 Nw_r()=0;

		protected:
			friend class Branch;
			friend class ForceTransJnt;
	/// axis of rotation for rev, axis of translation for trans. Parent Coordinates
	Vect3 m_axis_pk;
	/// axis of rotation for rev, axis of translation for trans. Child Coordinates
    Vect3 m_axis_k;

    /// return the modified Jacobian for constraint type P1,2 page 299 Nikravesh
    MatNxN getJacobianModP12(int ParentOrChild);
    VectN getGamaPoundP12();
    MatNxN getConstraintViolationP12() const;
    MatNxN getJacobianP12(int ParentOrChild);

    MatNxN getJacobianModP22(int ParentOrChild);
    MatNxN getJacobianP22(int ParentOrChild);
	MatNxN getConstraintViolationP22() const;
	VectN getGamaPoundP22();

    MatNxN getJacobianModN11a(int ParentOrChild);
    MatNxN getJacobianN11a(int ParentOrChild);
    MatNxN getConstraintViolationN11a() const;
    VectN getGamaPoundN11a();

    MatNxN getJacobianModN11b(int ParentOrChild);
    MatNxN getJacobianN11b(int ParentOrChild);
    MatNxN getConstraintViolationN11b() const;
    VectN getGamaPoundN11b();

    Vect3 getParentAxisInLocal() const {return m_axis_pk;};
    Vect3 getChildAxisInLocal() const {return m_axis_k;};
    Vect3 getParentAxisInGlobal() const {return m_parent->getRot()*m_axis_pk;};
    Vect3 getChildAxisInGlobal() const {return m_child->getRot()*m_axis_k;};

    virtual MatNxN getJacobian( int ParentOrChild)=0;

    /// calculate the vectors perpendicular to the axis that themselves must be perp.
    /// used for trans joint
    std::vector<Vect3> getHiHj() const;
};
};
#endif