#ifndef OMDOMDJOINT1DOF_H
#define OMDOMDJOINT1DOF_H

#include "JointAxis.h"

namespace OMD
{
	class Joint1DOF :
		public JointAxis
	{
	public:
		
		Joint1DOF(string const &name, BodyRigid *parent, vector<double> const &parent2joint,
			BodyRigid *child, vector<double> const &joint2child,vector<double> const &axisInParent,double q0 = 0., double u0 = 0. );

		Joint1DOF(string const &name, BodyRigid *parent,Vect3 parent2joint,
			BodyRigid *child, Vect3 joint2child,Vect3 axisInParent,double q0 = 0., double u0 = 0. );
		virtual ~Joint1DOF(void);

		///
		/// Do Position Kinematics
		/// @param t Simulation Time
		/// @return nothing
		///
		virtual void kinematicsPosition( double t ) = 0;

		/// implementation of equation 78 & 80 page 59
		virtual void ForwardSubstitute( double t );
		virtual void triangularize(double t );
		virtual std::vector<double> getState( );
		virtual unsigned int setState( std::vector<double> state_vector );
		virtual std::vector<double> getDot( );
		virtual unsigned int getStateSize();
		virtual Vect3 calcChildVel( double t )=0;
		virtual Vect3 calcChildAngVel( double t )=0;

		virtual Vect3 KinematicsAccelLin( double t ) = 0;
		virtual Vect3 KinematicsAccelAng( double t ) = 0;

		inline double GetQ() { return m_q; };
		inline double GetU() { return m_u; };
		void kinematics();
		///
		///Get the rotation matrix which will transform from Parent to Child
		///
		/// @return rotation matrix
		Mat3x3 GetRotFromParent2Child(){return pk_A_k;};
		virtual MatNxN getJacobianModified( int ParentOrChild )=0;
		virtual VectN getGamaPound()=0;
		virtual vector<double> getViolation()=0;
		///
		/// sorry for the terrible name but it's physical significance is ?
		/// this is equestion 41 page 51 of Anderson
		///
		virtual Vect3 Nv_r()=0;
		/// this is equestion 40 page 51 of Anderson
		virtual Vect3 Nw_r()=0;
	protected:
		friend class Branch;
		friend class ForceTransJntSpringDamp;
		///Rotation matrix converts vector from child in parent reference frame
		///Equation Notation: \f$ ^{p\left[k \right ]}{A}^k \f$
		Mat3x3 pk_A_k;
		// P underscore
		Vect6 m_P_;
		double m_sM;
		double m_invsM;
		Vect6 m_neg_inv_sM_sP;
		double m_q;	//position
		double m_u;	//velocity
		double m_qdot;	//derivative of position
		double m_udot;	//derivative of velocity
		//	/// axis of rotation for rev, axis of translation for trans. Parent Coordinates
		//	Vect3 m_axis_pk;
		//	/// axis of rotation for rev, axis of translation for trans. Child Coordinates
		//   Vect3 m_axis_k;


	private:
		//			virtual void Setup() = 0;
		Mat6x6 m_sSC; // shift matrix equation 58 59 & 60 page55
	};
};
#endif