#ifndef OMDOMDJOINT_H
#define OMDOMDJOINT_H

#include <string>
#include "OMD.h"
#include "BodyRigid.h"
#include <vector>

#define PARENT 0
#define CHILD 1

namespace OMD
{
	using namespace std;
	class Joint
	{
	public:
		///
		/// Constructor
		/// @param name Name of the joint
		///
		Joint(string name);
		///
		/// Constructor
		/// @param name Name of the joint
		/// @param parent BodyRigid which serves as the parent
		/// @param parent2joint Vector describing the location of the joint wrt the parent in parent coordinate
		/// @param child BodyRigid which serves as the child
		/// @param joint2child Vector describing the location of the child relative to the joint in parent coordinates
		///
		Joint(string name, BodyRigid *parent, std::vector<double> const &parent2joint, BodyRigid *child, std::vector<double> const &joint2child);

		///
		/// Constructor
		/// @param name Name of the joint
		/// @param parent BodyRigid which serves as the parent
		/// @param parent2joint Vector describing the location of the joint wrt the parent in parent coordinate
		/// @param child BodyRigid which serves as the child
		/// @param joint2child Vector describing the location of the child relative to the joint in parent coordinates
		///
		Joint(string name, BodyRigid *parent, Vect3 &parent2joint, BodyRigid *child, Vect3 &joint2child);
		virtual ~Joint(){};
		//	///
		//	/// Do Position Kinematics
		//	/// @param t Simulation Time
		//	/// @return nothing
		//	///
		//	virtual void kinematicsPosition( double t ) = 0;
		///
		/// Calculate the Velocity Child of the Joint Not a "Get"
		///
		/// @return Vect3
		///
		//	virtual Vect3 calcChildVel( double t )=0;
		//	virtual Vect3 calcChildAngVel( double t )=0;
		//	virtual Vect3 KinematicsAccelAng( double t ) = 0;
		//	virtual Vect3 KinematicsAccelLin( double t ) = 0;
		//	virtual void ForwardSubstitute( double t ) = 0;
		//	virtual void triangularize(double t ) = 0;
		//	virtual std::vector<double> getState( ) = 0;
		//	virtual unsigned int setState( std::vector<double> state_vector ) = 0;
		//	virtual std::vector<double> getDot ( ) = 0;
		//	virtual unsigned int getStateSize() = 0;
		//	void kinematics();
		///
		/// Set the Distance from Parent Center of mass to Joint in Parent coordinates
		///
		/// @param[in] distance in parent coordinates
		/// @return Nothing
		///
		void SetParent2Joint(Vect3 dist);
		///
		/// Set the Distrance from Joint to Child center of mass in child coordinates
		///
		/// @param[in] distance in child coordinates
		/// @return Nothing
		///
		void SetJoint2Child(Vect3 dist);
		///
		/// Return name string
		///
		/// @return Name
		string getName(){return m_name;};
		///
		///Get the parent body
		///
		/// @return pointer to parent body
		BodyRigid *getParent(){return m_parent;};
		bool isParent(BodyRigid * body) const;
		bool isChild(BodyRigid * body) const;
		bool isParentOrChild(BodyRigid * body) const;
		///
		///Get the child body
		///
		/// @return pointer to parent body
		BodyRigid *getChild(){return m_child;};
		Vect3 GetParentCG2ChildCG(){return m_gamma_pk;};

		Vect3 getParent2JointLocal() const {return r_pk_j;};
		Vect3 getParent2JointGlobal() const {return m_parent->getRot()*r_pk_j;};
		Vect3 getChild2JointLocal() const {return -m_r_j_k;};
		Vect3 getChild2JointGlobal() const {return m_child->getRot()*(-m_r_j_k);};
		virtual MatNxN getJacobianModified( int ParentOrChild)=0;
		virtual MatNxN getJacobian( int ParentOrChild)=0;
		virtual VectN getGamaPound()=0;

		/// return the modified Jacobian for constraint type S3 page 299 Nikravesh
		MatNxN getJacobianModS3(int ParentOrChild = PARENT);
		//{

		//	if (ParentOrChild == PARENT)
		//	{
		//		bool identity = true;
		//		MatNxN part1(3,3,identity);

		//		Mat3x3 Ai = m_parent->getRot();

		//		Vect3 sip = getParent2JointGlobal();
		//		Mat3x3 sip_s = skew(sip);

		//		MatNxN part2 = -1 *sip_s * Ai;

		//		/// horizontal concatinate
		//		MatNxN temp(part1.rows()+part2.rows(),part1.cols());
		//		temp << part1,part2;

		//		return temp;
		//	}
		//	else
		//	{
		//		bool identity = true;
		//		MatNxN part1(3,3,identity);
		//		part1 = -1*part1;

		//		Mat3x3 Aj = m_child->getRot();
		//		Vect3 sjp = getChild2JointGlobal();
		//		Mat3x3 sjp_s = skew(sjp);

		//		MatNxN part2 =  sjp_s * Aj;

		//		// horizontal concatinate
		//		MatNxN temp(part1.rows()+part2.rows(),part1.cols());
		//		temp << part1,part2;

		//		return temp;
		//	}
		//};
		VectN getGamaPoundS3();
		MatNxN getConstraintViolationS3() const;
		MatNxN getJacobianS3(int ParentOrChild = PARENT);
		///
		/// Violation of Joint Constraint
		///
		virtual std::vector<double> getViolation()=0;
		///
		/// page 201 table 7.1 Nikravesh
		///
		MatNxN getB(int ParentOrChild = PARENT);

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	protected:
		friend class Branch;
		/// page 189 Nikravesh
		unsigned int row2Drop(Vect3 si) const;
		string m_name;
		/// vector from p[k*] to j- fixed in 'A' (i.e. P[k]'s reference frame) \n
		/// Equation Notation: \f$ r^{Pr\left[k \right ]} \f$
		Vect3 r_pk_j;
		///vector from j+ to k* fixed in 'B' (i.e. k's reference frame)
		Vect3 m_r_j_k;
		///Rotation matrix converts vector from child to parent in initial configuration
		Mat3x3 pk_c_k0;

		///Rotation matrix converts vector from parent to child reference frame.
		///Equation Notation: \f$ ^k{A}^{p\left[k \right ]} \f$
		Mat3x3 k_A_pk;

		///Parent CG to Child CG in parent coordinates
		///Found on page 39 Figure 2.1.3 and
		///Found on page 52 equation 42 of Recursive Derivation of Explicit Equations ... Anderson \n
		///Equation Notation: \f$ ^{pk}\gamma \f$
		Vect3 m_gamma_pk;
		///Parent CG to Child CG in child coordinates
		///Found on page 39 Figure 2.1.3 and
		///Found on page 52 equation 42 of Recursive Derivation of Explicit Equations ... Anderson
		Vect3 m_gamma_k;

		///vector from j- to j+ , i.e. the joint location on parent ( p[k] ) to the child (k)
		/// in parent coordinates
		Vect3 m_r_jj;

		//	Matrix6 m_sSC; // shift matrix equation 58 59 & 60 page55
		Mat6x6 m_sT;
		//	Matrix6 m_sIhat_sSCT;

		BodyRigid *m_parent;
		BodyRigid *m_child;

	};
};
#endif
