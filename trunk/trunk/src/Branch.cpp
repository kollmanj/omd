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
#include "Branch.h"
#include <iostream>

namespace OMD
{
	Branch::Branch()
	{
		m_level = 0;		// by default it is 0
	}

	Branch::Branch(Branch *b0)
	{
		m_joints = b0->m_joints;
		m_level = b0->m_level;
	}

	Branch::Branch(Joint1DOF *joint, unsigned int i)
	{
		m_joints.push_back ( joint );
		m_level = i;		// by default it is 0
	}

	Branch::~Branch()
	{
	}


	void Branch::kinematics ( double t )
	{
		for ( unsigned int i = 0; i<m_joints.size(); ++i )
		{
		    //need to do this to the joints before moving on with bodies
			m_joints[i]->kinematicsPosition(t);

			BodyRigid *parent = m_joints[i]->getParent();
			BodyRigid *child = m_joints[i]->getChild();
			string childName = child->m_name;

			Mat3x3 Ap = parent->getRot();

			Mat3x3 ROT = Ap*m_joints[i]->GetRotFromParent2Child();
			child->setRot( ROT );

//			Mat3x3 PK_C_K =  m_joints[i]->GetRotFromParent2Child();

//			child->setPosition( parent->getRot() * ( parent->GetPosition () + m_joints[i]->GetParentCG2ChildCG() ));
			child->setPosition( parent->m_pos + parent->getRot() * (  m_joints[i]->GetParentCG2ChildCG() ));


			// Kane's method angular velocity   AngVel must come before linear velocity
			child->setAngularVelocity( m_joints[i]->calcChildAngVel( t ) );

			//Kane's method linear velocity
			child->setVelLocal (m_joints[i]->calcChildVel(t));

			//state dependent angular acceleration
			//child->setAlpha_t_k( m_joints[i]->KinematicsAccelAng( t ) );
			child->m_n_alpha_t_k = m_joints[i]->KinematicsAccelAng( t );
			//state dependent linear acceleration

			//child->setA_t_k( m_joints[i]->KinematicsAccelLin( t ) );
			child->m_n_a_t_k = m_joints[i]->KinematicsAccelLin( t );
			// this method is back in the Joint class not jointrevolute or jointtrans
			m_joints[i]->kinematics();
		}

	}

	void Branch::triangularize(double t)
	{
		//triangularize from bottom up
		std::vector< Joint1DOF *>::reverse_iterator rit;
		for (rit = m_joints.rbegin(); rit < m_joints.rend(); ++rit )
		{
			(*rit)->triangularize(t);
		}
	}

	void Branch::setBodyAccels()
	{
		std::vector< Joint1DOF *>::iterator it;
		Vect3 N_a_hat_Pk;
		N_a_hat_Pk << 0,0,0;

		Vect3 N_alpha_hat_Pk;
		N_alpha_hat_Pk << 0,0,0;
		
		for (it = m_joints.begin(); it < m_joints.end(); ++it)
		{
			Vect3 Nv_rk = (*it)->Nv_r();
			BodyRigid * child = (*it)->m_child;

			N_a_hat_Pk = (*it)->k_A_pk * N_a_hat_Pk; /// convert from parent to child coord.
			N_alpha_hat_Pk = (*it)->k_A_pk * N_alpha_hat_Pk; /// convert from parent to child coord.

			Mat3x3 k_A_pk = (*it)->k_A_pk;

			Vect3 childAccelLocal = Nv_rk * (*it)->m_udot + child->m_n_a_t_k;
			Vect3 child_n_alpha_t_k = child->m_n_alpha_t_k;
			Vect3 N_a_hat_k = N_a_hat_Pk + N_alpha_hat_Pk.cross((*it)->m_gamma_k) + Nv_rk * (*it)->m_udot;

			//child->setAcceleration(N_a_hat_k +  child->m_q*childAccelLocal);
			child->setAcceleration(child->m_q*(N_a_hat_k + child->m_n_a_t_k));

			/// Anderson equation 43 page 52
			Vect3 N_alpha_hat_k = N_alpha_hat_Pk + (*it)->Nw_r() * (*it)->m_udot; 

			//Vect3 childAngAccel = temp_ang + child_n_w_r_k * (*it)->m_udot + child->m_n_alpha_t_k;
			Vect3 childAngAccel = N_alpha_hat_k +  child->m_n_alpha_t_k;
			child->setLocalAngularAccel(childAngAccel);
						

			/// this stuff is for use in the next time around the loop
			N_alpha_hat_Pk = N_alpha_hat_k;  /// TODO might need to transform N_alpha_hat_Pk to child
			N_a_hat_Pk = N_a_hat_k;
		}
	}


	void Branch::ForwardSubstitute( double t )
	{
		//forward substitute top down
		std::vector< Joint1DOF *>::iterator it;
		for (it = m_joints.begin(); it < m_joints.end(); ++it )
		{
			(*it)->ForwardSubstitute(t);
		}
	}

	int Branch::findJoint(Joint *joint)
	{
		for ( unsigned int i = 0; i<m_joints.size(); ++i )
		{
			if (m_joints[i] == joint)
			{
				return i;
				break;
			}
		}
		return -1;
	}

	bool Branch::hasJoint(Joint *joint)
	{
		for ( unsigned int i = 0; i<m_joints.size(); ++i )
		{
			if (m_joints[i] == joint)
			{
				return true;
				break;
			}
		}
		return false;
	}

	bool Branch::hasBody(BodyRigid *body)
	{
		for ( unsigned int i = 0; i<m_joints.size(); ++i )
		{
			if ((m_joints[i]->getParent() == body) || (m_joints[i]->getChild() == body))
			{
				return true;
				break;
			}
		}
		return false;
	}

	void Branch::addJoint(Joint1DOF *joint)
	{
		m_joints.push_back ( joint );
	}

	void Branch::setLevel(unsigned int i)
	{
		m_level = i;
	}

	unsigned int Branch::getLevel()
	{
		return m_level;
	}

	std::vector<double> Branch::getState( )
	{
      std::vector<double> state_vector;
		std::vector< Joint1DOF *>::iterator it;
		for (it = m_joints.begin(); it < m_joints.end(); ++it )
		{
			std::vector<double> state_vector_jnt = (*it)->getState( );
//         state_size += state_vector_jnt.size();
         // append the state_vector_jnt to state_vector
         std::vector<double>::iterator it2;
         for (it2 = state_vector_jnt.begin(); it2 < state_vector_jnt.end(); ++it2)
         {
            state_vector.push_back(*it2);
         }
		}
		return state_vector;
	}
   unsigned int Branch::setState( std::vector<double> state_vector )
	{
      unsigned int state_size = 0;
		std::vector< Joint1DOF *>::iterator it;
		for (it = m_joints.begin(); it < m_joints.end(); ++it )
		{
         // get state size of the joint
         unsigned int jnt_state_size = (*it)->getStateSize();
         std::vector<double> jnt_state_vector;
         for (unsigned int i = state_size; i< jnt_state_size + state_size; i++)
         {
            jnt_state_vector.push_back( state_vector[i] );
         }
         (*it)->setState( jnt_state_vector );

         state_size += jnt_state_size;
		}
      return state_size;
	}
   unsigned int Branch::getStateSize()
   {
	    unsigned int statesize = 0;
	   	std::vector< Joint1DOF *>::iterator it;
		for (it = m_joints.begin(); it < m_joints.end(); ++it )
		{
			statesize += (*it)->getStateSize();
		}
		return statesize;
   }
	std::vector<double > Branch::getDot( )
	{

      std::vector<double> dot_vector;
		std::vector< Joint1DOF *>::iterator it;
		for (it = m_joints.begin(); it < m_joints.end(); ++it )
		{
			std::vector<double> dot_vector_jnt = (*it)->getDot( );
         std::vector<double>::iterator it2;
         for (it2 = dot_vector_jnt.begin(); it2 < dot_vector_jnt.end(); ++it2)
         {
            dot_vector.push_back(*it2);
         }
		}
		return dot_vector;

	}

	void Branch::print()
	{
		for ( unsigned int i = 0; i<m_joints.size(); ++i )
		{
			cout << "\tJoint " << i << " is: " << m_joints[i]->getName() << endl;
		}
	}
	unsigned int Branch::Size()
	{
		return m_joints.size();
	}

	Joint1DOF * Branch::getLastJoint()
	{
		return m_joints[m_joints.size()-1];
	}
	bool Branch::isLastBody(BodyRigid *body)
	{
		if ( m_joints[m_joints.size()-1]->getChild() == body )
		{
			return true;
		}
		else
		{
			return false;
		}
	}

}
