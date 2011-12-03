#include "JointRevolute.h"
#include <iostream>

namespace OMD
{
	JointRevolute::JointRevolute ( string const& name, BodyRigid* parent, vector<double> parent2joint, BodyRigid* child, vector<double> joint2child, vector<double> axis,double q0, double u0 ):
		Joint1DOF ( name, parent, parent2joint, child, joint2child, axis, q0, u0 )
	{
		m_P_ << m_axis_k,m_axis_k.cross(m_r_j_k);
	}

	JointRevolute::JointRevolute ( string const& name, BodyRigid* parent, Vect3 parent2joint, BodyRigid* child, Vect3 joint2child,Vect3 axis, double q0, double u0 ) : Joint1DOF ( name, parent, parent2joint, child, joint2child, axis, q0, u0 )
	{
		//m_P_ = Vect6(m_axis_k,m_axis_k.cross(m_r_j_k));
		m_P_ << m_axis_k,m_axis_k.cross(m_r_j_k);
	}

	JointRevolute::~JointRevolute()
	{
	}

	void JointRevolute::kinematicsPosition( double t )
	{
		//Matrix3 k0_A_k;	// trans from initial child to current child location

		Mat3x3 k0_A_k(AngleAxisd(m_q,m_axis_k));

//		Mat3x3 k0_A_k(m_axis_k,m_q);  // axis angle definition
		//k0_A_k.FromAxisAngle(m_axis_pk,m_q);

		// parent to child trans  pk_c_k0 initialized in 1DOF constructor
		pk_A_k =  pk_c_k0 * k0_A_k;

		k_A_pk = pk_A_k.transpose();

		// cg parent to child i.e. pk to k in parent i.e. pk
		m_gamma_pk = pk_A_k * m_r_j_k + r_pk_j;
		// cg paren to child i.e. pk to k in child i.e. k
		m_gamma_k = k_A_pk * m_gamma_pk;

	}
	Vect3 JointRevolute::calcChildVel( double t )
	{
		m_qdot = m_u;
		// velocity of joint in parent
		Vect3 vel_o_jnt_pk = m_parent->getVelocityLocal() + m_parent->getAngularVelocityLocal().cross(r_pk_j);
		Vect3 output = k_A_pk*vel_o_jnt_pk + m_child->getAngularVelocityLocal().cross(m_r_j_k);
		return output;
	}
	Vect3 JointRevolute::calcChildAngVel( double t )
	{
		//	Vect3 pk_omega_k = m_axis_pk * m_u;
		Vect3 k_omega_k = m_axis_k * m_u;
		//	Vect3 output = k_A_pk*m_parent->getAngVel() + pk_omega_k;
		Vect3 output = k_A_pk*m_parent->getAngularVelocityLocal() + k_omega_k;
		return output;
	}
	Vect3 JointRevolute::KinematicsAccelLin( double t )
	{
		Vect3 v1 = m_parent->m_n_a_t_k;
		Vect3 v2 = m_parent->m_n_alpha_t_k.cross(r_pk_j);
		Vect3 v3 = m_parent->getAngularVelocityLocal().cross(m_parent->getAngularVelocityLocal().cross(r_pk_j));
		Vect3 v4 = m_child->m_n_alpha_t_k.cross(m_r_j_k);
		Vect3 v5 = m_child->getAngularVelocityLocal().cross( m_child->getAngularVelocityLocal().cross(m_r_j_k));
		Vect3 output = k_A_pk*(v1+v2+v3) + v4 + v5;
		return output;
	}

	Vect3 JointRevolute::KinematicsAccelAng( double t )
	{
		/// FIXED LINE BELOW WITH LINE BELOW THAT!
		//Vect3 n_w_k_x_pk_w_k = m_child->getAngVel().crossProduct(m_axis_pk * m_u);
		Vect3 n_w_k_x_pk_w_k = m_child->getAngularVelocityLocal().cross(m_axis_k * m_u);
		Vect3 output = k_A_pk * m_parent->m_n_alpha_t_k    + n_w_k_x_pk_w_k;

		return output;
	}

	MatNxN JointRevolute::getJacobianModified(int ParentOrChild)
	{
		//// TODO !!!!! line below goes back in.  Line 2 lines below goes OUT!
		MatNxN s3iM = getJacobianModS3(ParentOrChild);
		//MatNxN s3iM = getJacobianModP12(ParentOrChild);
		MatNxN p12iM = getJacobianModP12(ParentOrChild);

		MatNxN out = concatV(s3iM, p12iM);
		return out;
	}

	vector<double> JointRevolute::getViolation()
	{
		vector<double> out;
		MatNxN p1_2 = getConstraintViolationP12();
		//    std::cout << "p1_2: " << p1_2 << std::endl;
		MatNxN s_3 = getConstraintViolationS3();
		//    std::cout << "s_3: " << s_3 << std::endl;

		out.push_back(s_3(0));
		out.push_back(s_3(1));
		out.push_back(s_3(2));
		out.push_back(p1_2(0));
		out.push_back(p1_2(1));

		return out;
	}

	MatNxN JointRevolute::getJacobian( int ParentOrChild )
	{
		MatNxN s3i = getJacobianS3(ParentOrChild);
		MatNxN p12i = getJacobianP12(ParentOrChild);

		MatNxN out = concatV(s3i,p12i);
		//std::cout << out << std::endl;
		return out;
	}

	VectN JointRevolute::getGamaPound()
	{
		VectN s3 = getGamaPoundS3();
		VectN p12 = getGamaPoundP12();

		VectN out = concatV(s3,p12);
		return out;
	}

	Vect3 JointRevolute::Nw_r()
	{
		return m_axis_k;
	}

	Vect3 JointRevolute::Nv_r()
	{
		//// translational acceleration
		//Vect3 parentAccel = m_parent->m_accel;
		//Vect3 pk_wldot = m_parent->m_wldot;
		//// acceleration at joint in parent
		//Vect3 pk_accel_atJoint = pk_wldot.cross(r_pk_j);
		//// accerlation at child in parent relative to joint

		//// Vector from joint to child in child:  m_r_j_k
		//Vect3 m_pk_r_j_k = k_A_pk.transpose() * m_r_j_k;

		//// accerlation at child in parent relative to joint
		//Vect3 temp = (m_axis_pk * m_udot).cross(m_pk_r_j_k);

		//Quat parentq = m_parent->m_q;

		//Vect3 child_accel = parentq * (pk_accel_atJoint + temp);


		////Mat3x3 childRot = m_child->getRot();
		////std::cout << "childRot: " << childRot << std::endl;
		////std::cout << "k_A_pk: " << k_A_pk << std::endl;
		//
		//m_child->setAcceleration(child_accel);
		////m_child->setLocalAngularAccel(

		//Vect3 a = m_parent->m_n_a_t_k;
		//Vect3 b = m_child->m_n_a_t_k;
		//Vect3 temp = m_axis_k.cross(m_r_j_k)*m_udot;
//		Vect3 childAccelLocal = m_axis_k.cross(m_r_j_k)*m_udot + m_child->m_n_a_t_k;
		return m_axis_k.cross(m_r_j_k);
	}
}