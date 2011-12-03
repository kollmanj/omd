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
#include "JointTranslational.h"

namespace OMD {

JointTranslational::JointTranslational(string const &name, BodyRigid * parent,
		vector<double> const &parent2joint,
		BodyRigid *child,
		vector<double> const &joint2child,
		vector<double> const &axis,
		double q0, double u0 )
: Joint1DOF(name,parent,parent2joint,child,joint2child,axis,q0, u0)
{
	//vector from j- to j+ , i.e. the joint location on parent ( p[k] ) to the child (k)
	// in parent coordinates
	m_r_jj = m_axis_pk * q0;

	// constant partial derivatives wrt u (velocity)
	m_P_ << Vect3(0.,0.,0.),m_axis_k;
	//m_P_ = Vect6(Vect3(0.,0.,0.),m_axis_k);

	pk_A_k = pk_c_k0;	// from k to p[k]
	k_A_pk= pk_A_k.transpose();	//from p[k] to k

	m_gamma0 = r_pk_j + pk_A_k * m_r_j_k;
}

JointTranslational::JointTranslational(string const &name, BodyRigid * parent,
		Vect3 parent2joint,
		BodyRigid *child,
		Vect3 joint2child,
		Vect3 axis,
		double q0 , double u0 )
 : Joint1DOF(name,parent,parent2joint,child,joint2child,axis,q0, u0)
{
	//vector from j- to j+ , i.e. the joint location on parent ( p[k] ) to the child (k)
	// in parent coordinates
	m_r_jj = m_axis_pk * q0;


	// constant partial derivatives wrt u (velocity)
	m_P_ << Vect3(0.,0.,0.),m_axis_k;
	//m_P_ = Vect6(Vect3(0.,0.,0.),m_axis_k);


	pk_A_k = pk_c_k0;	// from k to p[k]
	k_A_pk= pk_A_k.transpose();	//from p[k] to k

	m_gamma0 = r_pk_j + pk_A_k * m_r_j_k;
}

JointTranslational::~JointTranslational()
{
}

void JointTranslational::kinematicsPosition( double t)
{
	m_r_jj = m_axis_pk * m_q;

	// gama
	m_gamma_pk = m_gamma0 + m_r_jj;
	m_gamma_k = k_A_pk * m_gamma_pk;

}

Vect3 JointTranslational::calcChildVel( double t)
{
	m_qdot = m_u;
//	m_v1 = m_r_jj * m_u; //what?!
   m_v1 = m_axis_pk * m_u;
	// angular velocity of parent (omega)
	Vect3 w_parent =  m_parent->getAngularVelocityLocal();
	m_v2 = w_parent.cross( m_gamma_pk );
	Vect3 linear_velocity = m_v1 + m_v2;
	return (k_A_pk*(linear_velocity + m_parent->getVelocityLocal()));
}

Vect3 JointTranslational::calcChildAngVel( double t )
{

	// It's a translational joint, no rotation dof partial ang
	// vel is 0, just get parent angular vel and transform
	return k_A_pk*(m_parent->getAngularVelocityLocal());
}
Vect3 JointTranslational::KinematicsAccelLin( double t )
{
	Vect3 parent_ang_vel = m_parent->getAngularVelocityLocal();
	Vect3 a1 = parent_ang_vel.cross(m_v1)*2.;
	Vect3 a2 = m_parent->m_n_alpha_t_k.cross(m_gamma_pk);
	Vect3 a3 = parent_ang_vel.cross(m_v2);
	Vect3 output = m_parent->m_n_a_t_k + a1 + a2 + a3;


		////
//	std::cout << parent_ang_vel.x << parent_ang_vel.y << parent_ang_vel.z << std::endl;
//	std::cout << a1.x << " " <<a1.y << " "  <<a1.z << std::endl;
//	std::cout << a2.x << " " <<a2.y<< " " <<a2.z<< std::endl;
//	std::cout << a3.x<< " " <<a3.y<< " " <<a3.z << std::endl;
//	std::cout << output.x<<output.y<<output.z << std::endl;
	////

	return output;
}

Vect3 JointTranslational::KinematicsAccelAng( double t )
{
	return k_A_pk * m_parent->m_n_alpha_t_k;
}

MatNxN JointTranslational::getJacobianModified( int ParentOrChild )
{
	MatNxN p12 = getJacobianModP12(ParentOrChild);
	MatNxN p22 = getJacobianModP22(ParentOrChild);
	MatNxN n11 = getJacobianModN11b(ParentOrChild);

	//std::cout << "p12: " << p12 << std::endl;
	//std::cout << "p22: " << p22 << std::endl;
	//std::cout << "n11: " << n11 << std::endl;

	// concatenate vertically
	MatNxN temp = concatV(p12,p22);

	// concatenate vertically again
	MatNxN temp2 = concatV(temp, n11);
	return temp2;
}

MatNxN JointTranslational::getJacobian( int ParentOrChild )
{
    MatNxN p12 = getJacobianP12(ParentOrChild);
	MatNxN p22 = getJacobianP22(ParentOrChild);
	MatNxN n11 = getJacobianN11b(ParentOrChild);

	// concatenate vertically
	MatNxN temp = concatV(p12,p22);
	// concatenate vertically again
	MatNxN temp2 = concatV(temp,n11);
	//std::cout << "p12: " << std::endl << p12 << std::endl;
	//std::cout << "p22: " << std::endl << p22 << std::endl;
	//std::cout << "n11: " << std::endl << n11 << std::endl;
	return temp2;
}

vector<double> JointTranslational::getViolation()
{
		vector<double> out;
		MatNxN p12 = getConstraintViolationP12();
		MatNxN p22 = getConstraintViolationP22();
		MatNxN n11 = getConstraintViolationN11b();

		out.push_back(p12(0));
		out.push_back(p12(1));
		out.push_back(p22(0));
		out.push_back(p22(1));
		out.push_back(n11(0));

		return out;
}

VectN JointTranslational::getGamaPound()
{
		VectN p12 = getGamaPoundP12();
		VectN p22 = getGamaPoundP22();
		VectN n11 = getGamaPoundN11b();

		// vertically concatenate
		VectN temp = concatV(p12,p22);

		// vertically concatenate again
		VectN temp2 = concatV(temp,n11);

		return temp2;
}

Vect3 JointTranslational::Nv_r()
{
	return m_axis_k;
}

Vect3 JointTranslational::Nw_r()
{
	return Vect3(0,0,0);
}

}
