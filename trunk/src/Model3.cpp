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
#include "OMDConfig.h"
#include "Model3.h"
#include "BodyRigid.h"
#include "JointTranslational.h"
#include <iostream>
#include <list>

#include <math.h>

namespace OMD
{

	Model3::Model3():m_states(&m_rigidBodies)
	{
		m_time0 = 0;
		m_epsilon=1e-8; /// TODO allow user to set this
		m_fixedRigidBodyCount = 0;
	}
	Model3::~Model3()
	{
		// delete Joints
		for (std::vector<Joint *>::iterator i=m_joint.begin(); i!=m_joint.end(); ++i)
		{
			delete *i;
		}


		//		// delete bodies
		//		for (unsigned int i=0; i < m_bodies.size(); ++i)
		//		{
		//			BodyRigid *body = m_bodies[i];
		//			delete body;
		//		}

	}

	std::vector<double> Model3::getState( )
	{
		std::vector<double> state;
		/// get only independent states
		std::vector< int >::iterator it;
		std::vector<int> IndependentIndices = m_states.getIndependentIndices();

		if (IndependentIndices.empty()) /// time must be 0 have not solved for independents
		{
			solve(0,false);
		}

		std::vector<double> s = m_states.getIndependentStates();
		return s;
	}

	std::vector<double> Model3::solve( double t, bool storeAccels )
	{
		m_time1 = t;
		MatNxN allAccels = getAllAccels();

		std::vector< double > independentDots = m_states.getIndependentDots(allAccels);

		return independentDots;
	}

	void Model3::calcIndependentStates()
	{
		//std::vector<int> rows2drop = std::vector<int>();
		jacobianRows2Drop.clear();
		MatNxN jacobian = getJacobian();
		MatNxN jacobianM = getJacobianModified();
		m_states.calcIndependentStates(jacobian,jacobianM);
	}

	std::vector<double> Model3::getDot( )
	{
		MatNxN allAccels = getAllAccels();
		std::vector< double > independentDots = m_states.getIndependentDots(allAccels);
		return independentDots;
	}

	void Model3::setState(std::vector<double> state_vector )
	{
		/// part c from page 321 Nikravesh
		/// solve for dependents by Newton Raphson
		double deltat = m_time1 - m_time0;
		m_states.setStates(state_vector, deltat);

		// Page 68 Nikravesh
		bool done = false;  // this will be set to true if we have reduce error below m_epsilon
		unsigned int fail_count=0; // if this gets too high we failed to converge
		while ( !done )
		{
			MatNxN jacobian = getJacobian();
			//			std::cout << "jacobian: " << jacobian << std::endl;
			MatNxN pos_error = getConstraintViolation();
			//std::cout << "pos_error : " << pos_error << std::endl;

			done = true;	// this will be set back to false if error is > m_epsilon
			for (int i=0; i<pos_error.rows(); ++i)
			{
				if (fabs(pos_error(i,0))>m_epsilon)
				{
					done=false;
				}
			}

			//// TODO only solve for dependents that have errors too high
			if (!done) /// we had some errors that were too high
			{
				fail_count++;
				MatNxN PHI_u(0,0); // constraint matrix for dependent dofs

				std::vector< int >::iterator it1;
				std::vector< int >  dependentIndices = m_states.getDependentIndices();
				for ( it1 = dependentIndices.begin(); it1 < dependentIndices.end(); it1++)
				{
					MatNxN column = jacobian.col(*it1);
					// concatenate Horizontally
					MatNxN temp = PHI_u;
					PHI_u = concatH(temp,column);
				}

				if (PHI_u.rows() == 0 )
				{
				    int crap1 = 1;
				}
				// put the left and right side together for row reduction
				MatNxN rightside = -1*pos_error;
				MatNxN temp = PHI_u.fullPivLu().solve(rightside);

				//std::vector<double> pos = m_states.getDependentPos();
				//MatNxN dependentpos(pos);
				MatNxN dependentpos = m_states.getDependentPos();

				MatNxN xjplus1 = dependentpos + temp;   // new approximation

				m_states.setDependentPos(xjplus1);

				int crap2 = 1;

				if (fail_count > 20)  ///TODO: less arbitrary
				{
					done = true;
					std::cout << "failed to converge" << std::endl;
				}
			}
		}
		// Done with NewtonRaphson
		 // sets velocities
		stepC3();

		/// normalize quaternions and qdot
		unsigned int bodyCount = m_rigidBodies.size();
		for (unsigned int i=0; i < bodyCount; ++i)
		{
			BodyRigid *b = m_rigidBodies[i];
			if ( !(b->isFixed()))
			{
				//			// page 316 & 317 Nikravesh

				Quat q = b->m_q;
				//			Vector4 qdot = b->getQuatDot();
				q.normalize();
				b->m_q =  q;
				//			//double sigma = qdot.w*q.e0 + qdot.x*q.e1 + qdot.y*q.e2 + qdot.z*q.e3;
				//			//Vector4 qdot_ = qdot - sigma*q;
				//			//b->setQuatDot(qdot);
			}
		}

		m_time0 = m_time1;
	}

	Vect3 Model3::getBodyPosition( string bodyname )
	{
		Vect3 position(0,0,0);
		string name;
		std::vector< BodyRigid *>::reverse_iterator rit;
		for (rit = m_rigidBodies.rbegin(); rit < m_rigidBodies.rend(); ++rit )
		{
			if (bodyname == (*rit)->m_name)
			{
				position = (*rit)->m_pos;
			}
		}
		return position;
	}

	Mat3x3 Model3::getBodyRot( string bodyname )
	{
		Mat3x3 rot;
		string name;
		std::vector< BodyRigid *>::reverse_iterator rit;
		/// TODO: do something if can't find body
		for (rit = m_rigidBodies.rbegin(); rit < m_rigidBodies.rend(); ++rit )
		{
			if (bodyname == (*rit)->m_name)
			{
				rot = (*rit)->getRot();
			}
		}
		return rot;
	}


	unsigned int Model3::getStateSize()
	{
		/// TODO: either fix this or throw it out
		return 1;
	}

	MatNxN Model3::getAllAccels()
	{
		// construct mass Matrix
		unsigned int bodyCount = m_rigidBodies.size();
		for (unsigned int i=0; i < bodyCount; ++i)
		{
			BodyRigid *body = m_rigidBodies[i];
			if (!(body->isFixed()))   /// TODO does this loop do anything, what was I thinking?
			{
				double m = body->m_mass;
				Mat3x3 M;
				M << m,0,0,
					 0,m,0,
					 0,0,m;

				concatD(massMatrix,M);
				Mat3x3 I = body->m_inertia;
				concatD(massMatrix,I);
			}
			// reset force accum
			body->forceAccumReset();
		}


		for (vector<Force*>::iterator i=m_forces.begin(); i!=m_forces.end(); ++i)
		{
			//Force *force = i->second;
			Force *force = *i;
			force->apply(m_time0); ///TODO: do I pass in m_time0 or m_time1
		}
		//	    /// construct forceAndTorqueVector
		//forceAndTorque.clear();
		forceAndTorque.resize(0,0);
		for (unsigned int i=0; i < bodyCount; ++i)
		{
			BodyRigid *body = m_rigidBodies[i];
			if (!(body->isFixed()))
			{
				//Vect3 bodyForce = body->getRot()*body->getAppliedForce();
				Vect3 bodyForce = body->getAppliedForce();
				Vect3 bodyTorque = body->getAppliedTorque();
				forceAndTorque = concatV(forceAndTorque,bodyForce);
				forceAndTorque = concatV(forceAndTorque,bodyTorque);
			}
		}
		//        std::cout << "forceAndTorque: " << forceAndTorque << std::endl;
		/// construct modified Jacobian and gama#
		MatNxN jacobianM = getJacobianModified();
		VectN gamaP = getGamaP();

		//std::cout << "massMatrix: "<< std::endl << massMatrix << std::endl;
		//std::cout << "jacobianM: "<< std::endl << jacobianM << std::endl;

		//
		//		// assemble the matrices we have for left matrix in equation 11.49 p 297 Nikravesh
		MatNxN LS = concatH(massMatrix,jacobianM.transpose());
		unsigned int rows = jacobianM.rows();
		unsigned int cols = jacobianM.rows();
		MatNxN zeros4LS(rows,cols);
		zeros4LS.fill(0);
		MatNxN LSBottom = concatH(jacobianM, zeros4LS);
		//
		LS = concatV(LS, LSBottom);
		//
		MatNxN b;
		// get the b for equation 11.40 page 296 Nikravesh
		for (unsigned int i=0; i < bodyCount; ++i)
		{
			BodyRigid *body = m_rigidBodies[i];
			if (!(body->isFixed()))
			{
				b = concatV(b,0); b = concatV(b, 0); b = concatV(b, 0);
				Vect3 omega = body->getAngularVelocityLocal();
				Mat3x3 omega_s = skew(omega);
				Vect3 r = omega_s * body->m_inertia *omega;
				b = concatV(b,r);
			}
		}

		//std::cout << "b: " << b << std::endl;
		MatNxN zeros4RS(rows,1);
		zeros4RS.fill(0);
		MatNxN bandZero = concatV(b, zeros4RS);

		// do right side
		MatNxN RS = forceAndTorque;
		RS = concatV(RS, gamaP) - bandZero;

		MatNxN allAccels = LS.fullPivLu().solve(RS);
		//std::cout << "AllAccels: " << allAccels << std::endl;

		return allAccels;
	}
	MatNxN Model3::getJacobian()
	{
		MatNxN jacobian;
		std::vector< Joint *>::iterator it;

		for (it = m_joint.begin(); it < m_joint.end(); ++it )
		{
		    MatNxN jac4Jnt;
			BodyRigid *parent = (*it)->getParent();
			BodyRigid *child = (*it)->getChild();
			MatNxN a = (*it)->getJacobian(PARENT);
			MatNxN b = (*it)->getJacobian(CHILD);

			// add columns of zeros for bodies not included in the jnts
			MatNxN z(a.rows(),7); // zeros to append
			z.fill(0);
			std::vector< BodyRigid * >::iterator it2;
			for ( it2 = m_rigidBodies.begin(); it2 < m_rigidBodies.end(); ++it2 )
			{
				if (!((*it2)->isFixed()))
				{
					if ( (*it2 != parent)  && (*it2 != child))
					{
						jac4Jnt = concatH(jac4Jnt, z);		
					}
					else if (*it2 == parent)
					{
						jac4Jnt = concatH(jac4Jnt,a);
					}
					else
					{
						jac4Jnt = concatH(jac4Jnt, b);
					}
				}
			}
			// 
			jacobian = concatV(jacobian, jac4Jnt);
		}

		int bodycount = m_rigidBodies.size();
		std::vector< BodyRigid * >::iterator it2;
		int i = 0;
		for ( it2 = m_rigidBodies.begin(); it2 < m_rigidBodies.end(); ++it2 )
		{
		    if (!((*it2)->isFixed()))
            {
			Quat q = (*it2)->m_q;
			MatNxN QuatComponent(1,7*(bodycount-m_fixedRigidBodyCount));
			QuatComponent.fill(0);
			QuatComponent(0,i*7+3) = 2.0*q.w();
			QuatComponent(0,i*7+4) = 2.0*q.x();
			QuatComponent(0,i*7+5) = 2.0*q.y();
			QuatComponent(0,i*7+6) = 2.0*q.z();
			jacobian = concatV(jacobian, QuatComponent);
			}
			else
			{
				break;  // all the fixed bodies are at the end
			}
			i++;
		}

 //       std::cout << "Jacobian: " << jacobian << std::endl;
		return jacobian;
	}

	MatNxN Model3::getJacobianModified( )
	{
		MatNxN jacobianM;

		/// construct modified Jacobian
		std::vector< Joint *>::iterator it;
		for (it = m_joint.begin(); it < m_joint.end(); ++it )
		{
		    MatNxN jac4Jnt;
			//int rows = jacobianM.rows();
			BodyRigid *parent = (*it)->getParent();
			BodyRigid *child = (*it)->getChild();
			MatNxN a = (*it)->getJacobianModified(PARENT);
			MatNxN b = (*it)->getJacobianModified(CHILD);

			// add columns of zeros for bodies not included in the jnts
			MatNxN z(a.rows(),6); // zeros to append
			z.fill(0);
			std::vector< BodyRigid * >::iterator it2;
			for ( it2 = m_rigidBodies.begin(); it2 < m_rigidBodies.end(); ++it2 )
			{
				if (!((*it2)->isFixed()))
				{
					if ( (*it2 != parent)  && (*it2 != child))
					{
						// concatenate Horizontally
						MatNxN temp = jac4Jnt;
						jac4Jnt = concatH(temp,z);
					}
					else if (*it2 == parent)
					{
						// concatenate Horizontally
						MatNxN temp = jac4Jnt;
						jac4Jnt = concatH(temp,a);
					}
					else
					{
						// concatenate Horizontally
						MatNxN temp = jac4Jnt;
						jac4Jnt = concatH(temp,b);
					}
				}
			}

			// concatenate vertically
			MatNxN temp = jacobianM;
			jacobianM = concatV(temp,jac4Jnt);
		}
		
		return jacobianM;
	}

	VectN Model3::getGamaP()
	{
		VectN gamaP;
		/// construct modified Jacobian and gama#
		std::vector< Joint *>::iterator it;
		for (it = m_joint.begin(); it < m_joint.end(); ++it )
		{
			int rows = gamaP.rows();
			VectN gpv = (*it)->getGamaPound();
			// concatenate vertically
			MatNxN temp = gamaP;
			gamaP = concatV(temp, gpv);
		}
		return gamaP;
	}

	MatNxN Model3::NewtonRaphson(double t)
	{

		/// TODO: just a place holder
		MatNxN result;
		return result;
	}

	MatNxN Model3::getConstraintViolation()
	{
		//std::vector< double > out;
		MatNxN out;
		std::vector< Joint *>::iterator it;
		for (it = m_joint.begin(); it < m_joint.end(); ++it )
		{
			std::vector< double > joint_error = (*it)->getViolation();
			std::vector< double >::iterator it2;
			for ( it2 = joint_error.begin(); it2 < joint_error.end(); it2++)
			{
				MatNxN temp = out;
				out = concatV(out, *it2); // concatenate
			}

		}

//		 put the quaternion constraint for each body in there
				std::vector< BodyRigid *>::iterator it2;
				for ( it2 = m_rigidBodies.begin(); it2< m_rigidBodies.end(); it2++)
				{
				    if (!((*it2)->isFixed()))
                    {
					BodyRigid *b = (*it2);
					Quat q = b->m_q;

					MatNxN temp = out;

					double quatError = q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z() -1;
					out = concatV(temp, quatError);
                    }
				}
		return out;
	}

	
	void Model3::stepC3()
	{
		//      makeJacobian(t);
		MatNxN jacobianM = getJacobianModified();
		MatNxN PHI_u(0,0);   // start off with zero size and concatenate
		MatNxN PHI_v(0,0);   // start off with zero size and concatenate

		std::vector< int >::iterator it1;
		std::vector<int> dependentIndices2 = m_states.getDependentIndices2();
		for ( it1 = dependentIndices2.begin(); it1 < dependentIndices2.end(); it1++)
		{
			MatNxN column = jacobianM.col(*it1);
			//PHI_u = PHI_u.concatH(column);

			//  concatenate horizontally
			MatNxN temp = PHI_u;
			PHI_u = concatH(temp, column);
		}

		//      std::cout << "Jacobian: " << jacobian << std::endl;
		//      std::cout << "PHI_u: " << PHI_u << std::endl;
		MatNxN independent_vel(m_states.getIndependentIndices2().size(),1);
		int j = 0;
		std::vector<int> independentIndices2 = m_states.getIndependentIndices2();
		for ( it1 = independentIndices2.begin(); it1 < independentIndices2.end(); it1++)
		{
			independent_vel(j,0) = m_states.getDot2(*it1);
			MatNxN column = jacobianM.col(*it1);
			//  concatenate horizontally
			MatNxN temp = PHI_v;
			PHI_v = concatH(temp,column);
			j++;
		}
		//      std::cout << "PHI_v: " << PHI_v << std::endl;
		MatNxN temp4 = -1*PHI_v*independent_vel;

		//MatNxN udot = PHI_u.solve(temp4);
		MatNxN udot = PHI_u.fullPivLu().solve(temp4);  // TODO check other solution schemes


		//        std::cout << "udot: " << udot << std::endl;

		//        std::cout << "udot : " << udot << std::endl;
		m_states.setDependentVel(udot);
		//std::vector< int >::iterator it4;
		//j = 0;
		//for ( it4 = m_states.getDependentIndices().begin(); it4 < m_states.getDependentIndices().end(); it4++ )
		//{
		//   m_states.setDot( *it4, udot[j][0]);
		//   j++;
		//}

	}


	////////// ADDING STUFF ///////////////////////

	JointTranslational* Model3::addJointTrans(	string const &name,
		BodyRigid * parent,
		Vect3 parent2joint,
		BodyRigid *child,
		Vect3 axis,
		double q0,
		double u0)
	{
		// determine joint2child based on where stuff is
		// location of joint in global
		Vect3 jointLg = parent->getPosition(parent2joint);
		// child 2 joint 2 local
		Vect3 c2jl = child->getPointInLocal(jointLg);
		Vect3 j2cl = -c2jl;

		//JointTranslational *j = new JointTranslational(name, parent, parent2joint, child, joint2child, axis, q0, u0);
		JointTranslational *j = new JointTranslational(name, parent, parent2joint, child, j2cl, axis, q0, u0);
		BodyRigid * p = j->getParent();
		BodyRigid * c = j->getChild();
        m_joint.push_back(j);
		return j;
	}

	JointTranslational* Model3::addJointTrans(string const &name, BodyRigid * parent,
			vector<double> parent2joint,
			BodyRigid *child,
			vector<double> axis,
			double q0, double u0 )
	{
		// determine joint2child based on where stuff is
		// location of joint in global
		Vect3 parent2joint_(parent2joint[0], parent2joint[2], parent2joint[3]);
		Vect3 axis_(axis[0],axis[1],axis[2]);
		Vect3 jointLg = parent->getPosition(parent2joint_);
		// child 2 joint 2 local
		Vect3 c2jl = child->getPointInLocal(jointLg);
		Vect3 j2cl = -c2jl;

		//JointTranslational *j = new JointTranslational(name, parent, parent2joint, child, joint2child, axis, q0, u0);
		JointTranslational *j = new JointTranslational(name, parent, parent2joint_, child, j2cl, axis_, q0, u0);
		BodyRigid * p = j->getParent();
		BodyRigid * c = j->getChild();
        m_joint.push_back(j);
		return j;
	}

	JointTranslational* Model3::addJointTrans(	string const &name,
		std::string parentname,
		Vect3 parent2joint,
		std::string childname,
		Vect3 axis,
		double q0,
		double u0)
	{
		// TODO: if BodyRigid does not exist fail gracefully
		BodyRigid *parent = getBody(parentname);
		BodyRigid *child = getBody(childname);

		// determine joint2child based on where stuff is
		// location of joint in global
		Vect3 jointLg = parent->getPosition(parent2joint);
		// child 2 joint 2 local
		Vect3 c2jl = child->getPointInLocal(jointLg);
		Vect3 j2cl = -c2jl;

		JointTranslational *j = new JointTranslational(name, parent, parent2joint, child, j2cl, axis, q0, u0);
		BodyRigid * p = j->getParent();
		BodyRigid * c = j->getChild();
        m_joint.push_back(j);
		return j;
	}

	JointTranslational* Model3::addJointTrans(string const &name, std::string parentname,
			vector<double> parent2joint,
			std::string childname,
			vector<double> axis,
			double q0, double u0)
	{
		// TODO: if BodyRigid does not exist fail gracefully
		BodyRigid *parent = getBody(parentname);
		BodyRigid *child = getBody(childname);
		
		// determine joint2child based on where stuff is
		// location of joint in global
		Vect3 parent2joint_(parent2joint[0],parent2joint[1],parent2joint[2]);
		Vect3 axis_(axis[0],axis[1],axis[2]);
		Vect3 jointLg = parent->getPosition(parent2joint_);
		// child 2 joint 2 local
		Vect3 c2jl = child->getPointInLocal(jointLg);
		Vect3 j2cl = -c2jl;

		JointTranslational *j = new JointTranslational(name, parent, parent2joint_, child, j2cl, axis_, q0, u0);
		BodyRigid * p = j->getParent();
		BodyRigid * c = j->getChild();
        m_joint.push_back(j);
		return j;
	}


	JointRevolute* Model3::addJointRevolute(	string const& name,
		BodyRigid* parent,
		Vect3 parent2joint,
		BodyRigid* child,
		Vect3 axis,
		double q0,
		double u0)
	{
		Mat3x3 Ai = parent->getRot();
		Mat3x3 Aj = child->getRot();

		Vect3 jntLocationGlobal = Ai*parent2joint + parent->m_pos;

		Vect3 joint2child = Aj.transpose()*(child->m_pos - jntLocationGlobal);
		JointRevolute *j = new JointRevolute( name, parent, parent2joint, child, joint2child, axis, q0, u0);
		m_joint.push_back(j);

		return j;
	}

	JointRevolute* Model3::addJointRevolute(	string const& name,
			BodyRigid * parent,
			vector<double> parent2joint,
			BodyRigid* child,
			vector<double> axis,
			double q0,
			double u0)
	{
		Vect3 axis_(axis[0],axis[1],axis[2]);

		Mat3x3 Ai = parent->getRot();
		Mat3x3 Aj = child->getRot();
		
		Vect3 parent2joint_(parent2joint[0],parent2joint[1],parent2joint[2]);
		Vect3 jntLocationGlobal = Ai*parent2joint_ + parent->m_pos;

		Vect3 joint2child = Aj.transpose()*(child->m_pos - jntLocationGlobal);
		JointRevolute *j = new JointRevolute( name, parent, parent2joint_, child, joint2child, axis_, q0, u0);
		m_joint.push_back(j);

		return j;
	}

	JointRevolute* Model3::addJointRevolute(	string const& name,
			std::string parentname,
			vector<double> parent2joint,
			std::string childname,
			vector<double> axis,
			double q0,
			double u0)
	{
		Vect3 axis_(axis[0],axis[1],axis[2]);

		// TODO: if BodyRigid does not exist fail gracefully
		BodyRigid *parent = getBody(parentname);
		BodyRigid *child = getBody(childname);

		Mat3x3 Ai = parent->getRot();
		Mat3x3 Aj = child->getRot();
		
		Vect3 parent2joint_(parent2joint[0],parent2joint[1],parent2joint[2]);
		Vect3 jntLocationGlobal = Ai*parent2joint_ + parent->m_pos;

		Vect3 joint2child = Aj.transpose()*(child->m_pos - jntLocationGlobal);
		JointRevolute *j = new JointRevolute( name, parent, parent2joint_, child, joint2child, axis_, q0, u0);
		m_joint.push_back(j);

		return j;
	}

	JointCylindrical* Model3::addJointCylindrical(	string const& name,
		BodyRigid* parent,
		Vect3 parent2joint,
		BodyRigid* child,
		Vect3 axis,
		double q0,
		double u0)
	{
		Mat3x3 Ai = parent->getRot();
		Mat3x3 Aj = child->getRot();

		Vect3 jntLocationGlobal = Ai*parent2joint + parent->m_pos;

		Vect3 joint2child = Aj.transpose()*(child->m_pos-jntLocationGlobal);
		JointCylindrical *j = new JointCylindrical( name, parent, parent2joint, child, joint2child, axis, q0, u0);
		m_joint.push_back(j);

		return j;
	}

	JointCylindrical* Model3::addJointCylindrical(	string const& name,
			BodyRigid * parent,
			vector<double> parent2joint,
			BodyRigid* child,
			vector<double> axis,
			double q0,
			double u0)
	{
		Mat3x3 Ai = parent->getRot();
		Mat3x3 Aj = child->getRot();
		Vect3 parent2joint_(parent2joint[0],parent2joint[1],parent2joint[2]);
		Vect3 jntLocationGlobal = Ai*parent2joint_ + parent->m_pos;

		Vect3 joint2child = Aj.transpose()*(child->m_pos-jntLocationGlobal);
		
		Vect3 axis_(axis[0],axis[1],axis[2]);
		JointCylindrical *j = new JointCylindrical( name, parent, parent2joint_, child, joint2child, axis_, q0, u0);
		m_joint.push_back(j);

		return j;
	}

	JointSpherical* Model3::addJointSpherical( string const& name,
			BodyRigid * parent,
			Vect3 parent2joint,
			BodyRigid* child
			)
	{
		Mat3x3 Ai = parent->getRot();
		Mat3x3 Aj = child->getRot();

		Vect3 jntLocationGlobal = Ai*parent2joint + parent->m_pos;

		Vect3 joint2child = Aj.transpose()*(child->m_pos-jntLocationGlobal);

		JointSpherical *j = new JointSpherical( name, parent, parent2joint, child, joint2child);
		m_joint.push_back(j);

		return j;
	}

	JointSpherical* Model3::addJointSpherical( string const& name,
			BodyRigid * parent,
			vector<double> parent2joint,
			BodyRigid* child
			)
	{
		Mat3x3 Ai = parent->getRot();
		Mat3x3 Aj = child->getRot();
		Vect3 parent2joint_(parent2joint[0],parent2joint[1],parent2joint[2]);

		Vect3 jntLocationGlobal = Ai*parent2joint_ + parent->m_pos;

		Vect3 joint2child = Aj.transpose()*(child->m_pos-jntLocationGlobal);

		JointSpherical *j = new JointSpherical( name, parent, parent2joint_, child, joint2child);
		m_joint.push_back(j);

		return j;
	}

	JointUniversal* Model3::addJointUniversal( string const& name, BodyRigid* parent, Vect3 parent2joint, Vect3 pAxis, BodyRigid* child,Vect3 cAxis)
	{
		// determine joint2child based on where stuff is
		// location of joint in global
		Vect3 jointLg = parent->getPosition(parent2joint);
		// child 2 joint 2 local
		Vect3 c2jl = child->getPointInLocal(jointLg);
		Vect3 j2cl = -c2jl;

		JointUniversal *j = new JointUniversal( name, parent, parent2joint, pAxis, child, j2cl, cAxis);
		m_joint.push_back(j);

		return j;
	}
	JointUniversal* Model3::addJointUniversal( string const& name,
                                            BodyRigid* parent,
                                            vector<double> parent2joint,
                                            vector<double> pAxis,
                                            BodyRigid* child,
                                            vector<double> cAxis)
	{
		// determine joint2child based on where stuff is
		// location of joint in global
		Vect3 parent2joint_(parent2joint[0],parent2joint[1],parent2joint[2]);
		Vect3 pAxis_(pAxis[0],pAxis[1],pAxis[2]);
		Vect3 cAxis_(cAxis[0],cAxis[1],cAxis[2]);
		Vect3 jointLg = parent->getPosition(parent2joint_);
		// child 2 joint 2 local
		Vect3 c2jl = child->getPointInLocal(jointLg);
		Vect3 j2cl = -c2jl;

		JointUniversal *j = new JointUniversal( name, parent, parent2joint_, pAxis_, child, j2cl, cAxis_);
		m_joint.push_back(j);

		return j;
	}
	JointUniversal* Model3::addJointUniversal( string const& name,
                                           string const& parentname,
                                            vector<double> parent2joint,
                                            vector<double> pAxis,
                                            string const& childname,
                                            vector<double> cAxis)
	{	
		BodyRigid *parent = getBody(parentname);
		BodyRigid *child = getBody(childname);
		// determine joint2child based on where stuff is
		// location of joint in global
		Vect3 parent2joint_(parent2joint[0],parent2joint[1],parent2joint[2]);
		Vect3 pAxis_(pAxis[0],pAxis[1],pAxis[2]);
		Vect3 cAxis_(cAxis[0],cAxis[1],cAxis[2]);
		Vect3 jointLg = parent->getPosition(parent2joint_);
		// child 2 joint 2 local
		Vect3 c2jl = child->getPointInLocal(jointLg);
		Vect3 j2cl = -c2jl;

		JointUniversal *j = new JointUniversal( name, parent, parent2joint_, pAxis_, child, j2cl, cAxis_);
		m_joint.push_back(j);

		return j;
	}

	BodyRigid* Model3::addBodyRigid(std::string const &name, 
									double const &mass, 
									Mat3x3 const &inertia, 
									Vect3 const &pos, 
									Quat const &q, 
									Vect3 const &vel, 
									Vect3 const &wl, 
									bool const &fixed)
	{
		BodyRigid *b = new BodyRigid(name,mass,inertia,pos,q,vel,wl,fixed);

		// if the body at the end is fixed and this one isn't put the non-fixed body in first
		// bodies that are fixed go at the end
		if ((m_rigidBodies.empty()) || (!m_rigidBodies.back()->isFixed()))
		{
			m_rigidBodies.push_back ( b );
		}
		else
		{
			m_rigidBodies.insert(m_rigidBodies.end()-1,b);
		}
		double bodyMass = b->m_mass;
		Mat3x3 bodyMassMat;
		bodyMassMat <<	bodyMass,0,0,
						0,bodyMass,0,
						0,0,bodyMass;

		if (!fixed)
		{
			Mat3x3 bodyInertia =b->m_inertia;
			massMatrix = concatD(massMatrix,bodyMassMat);
			massMatrix = concatD(massMatrix, bodyInertia);
		}
		else
		{
			m_fixedRigidBodyCount ++;
		}
		return b;
	}

	BodyRigid* Model3::addBodyRigid (  string name,
		double mass,
		Mat3x3 inertia,
		Vect3 position,
		Mat3x3 orientation,
		Vect3 initialVelLocal,
		Vect3 initialAngVelLocal,
		bool fixed)
	{
		Quat q(orientation);
		BodyRigid *b = new BodyRigid(name,mass,inertia,position,q,initialVelLocal,initialAngVelLocal,fixed);

		// if the body at the end is fixed and this one isn't put the non-fixed body in first
		// bodies that are fixed go at the end
		if ((m_rigidBodies.empty()) || (!m_rigidBodies.back()->isFixed()))
		{
			m_rigidBodies.push_back ( b );
		}
		else
		{
			m_rigidBodies.insert(m_rigidBodies.end()-1,b);
		}
		double bodyMass = b->m_mass;
		Mat3x3 bodyMassMat;
		bodyMassMat <<	bodyMass,0,0,
						0,bodyMass,0,
						0,0,bodyMass;

		if (!fixed)
		{
			Mat3x3 bodyInertia =b->m_inertia;
			massMatrix = concatD(massMatrix,bodyMassMat);
			massMatrix = concatD(massMatrix, bodyInertia);
		}
		else
		{
			m_fixedRigidBodyCount ++;
		}
		return b;
	}

	BodyRigid* Model3::addBodyRigid(std::string const &name, double const &mass, std::vector<double> const &inertia,
							std::vector<double> const &pos, std::vector<double> q, std::vector<double> const &vel, 
							std::vector<double> const &wl, bool const &fixed)
	{
		Mat3x3 i; 
		i		<<	inertia[0], inertia[1], inertia[2],
					inertia[3],	inertia[4],	inertia[5],
					inertia[6],	inertia[7], inertia[8];

		Quat quat(q[0],q[1],q[2],q[3]);
		Vect3 position(pos[0],pos[1],pos[2]);
		Vect3 initialVelLocal(vel[0],vel[1],vel[2]);
		Vect3 initialAngVelLocal(wl[0],wl[1],wl[3]);
		BodyRigid *b = new BodyRigid(name,mass,i,position,quat,initialVelLocal,initialAngVelLocal,fixed);

		// if the body at the end is fixed and this one isn't put the non-fixed body in first
		// bodies that are fixed go at the end
		if ((m_rigidBodies.empty()) || (!m_rigidBodies.back()->isFixed()))
		{
			m_rigidBodies.push_back ( b );
		}
		else
		{
			m_rigidBodies.insert(m_rigidBodies.end()-1,b);
		}
		double bodyMass = b->m_mass;
		Mat3x3 bodyMassMat;
		bodyMassMat <<	bodyMass,0,0,
						0,bodyMass,0,
						0,0,bodyMass;

		if (!fixed)
		{
			Mat3x3 bodyInertia =b->m_inertia;
			massMatrix = concatD(massMatrix,bodyMassMat);
			massMatrix = concatD(massMatrix, bodyInertia);
		}
		else
		{
			m_fixedRigidBodyCount ++;
		}
		return b;
	}

	ForceRevJnt * Model3::addForceRevJnt(string const& name,
		JointRevolute *jnt,
		double trq)
	{
		ForceRevJnt * f = new ForceRevJnt(name,jnt,trq);
		m_forces.push_back(f);
		return f;
	}

	ForceRevJntSpringDamp * Model3:: addForceRevJntSpringDamp(string const& name,
		JointRevolute *jnt,
		double k,
		double c,
		double fl)
	{
		ForceRevJntSpringDamp * f = new ForceRevJntSpringDamp(name,jnt,k,c,fl);
		m_forces.push_back(f);
		return f;
	}

	ForceRevJntPIDCurve2D * Model3::addForceRevJntPIDCurve2D(string const& name,
		JointRevolute * jnt,
		double p,
		double i,
		double d,
		Curve2DSine *curve)
	{
		// copy incomming curve
		// curve was already added
		//m_curve2Ds.push_back(curve); // add curve so we can keep it around and delete it in the end
		ForceRevJntPIDCurve2D *f = new ForceRevJntPIDCurve2D(name,jnt,p,i,d,curve);
		m_forces.push_back(f);
		return f;
	}

//#ifdef USE_OGRE	// use with ogre
//	ForceCollisionDynWorld * Model3::addForceCollisionDynWorld ( string const& name, double stiff, double damp, double frict, double thresh )
//	{
//		bool nameAlreadyUsed = searchForcesForName(name);
//		if (!nameAlreadyUsed)
//		//if (m_forces.find(name) == m_forces.end())
//		{
//			ForceCollisionDynWorld *f = new ForceCollisionDynWorld ( name, stiff, damp, frict, thresh );
//			m_forces.push_back(f);
//			//m_forces[f->getName()] = f;
//			return f;
//		}
//		else
//			return NULL;
//	}
//#endif
//
//#ifdef USE_BULLET
//	ForceContact * Model3::addForceContact ( string const& name, btCollisionObjectArray objects,double stiff, double damp, double frict, double thresh )
//	{
//		bool nameAlreadyUsed = searchForcesForName(name);
//		if (!nameAlreadyUsed)
//		//if (m_forces.find(name) == m_forces.end())
//		{
//			ForceContact *f = new ForceContact ( name, objects, stiff, damp, frict, thresh );
//			//m_forces[f->getName()] = f;
//			m_forces.push_back(f);
//			return f;
//		}
//		else
//			return NULL;
//	}
//	ForceContact * Model3::addForceContact ( string const& name, btCollisionWorld *collisionWorld, double stiff, double damp, double frict, double thresh)
//	{
//		bool nameAlreadyUsed = searchForcesForName(name);
//		if (!nameAlreadyUsed)
//		//if (m_forces.find(name) == m_forces.end())
//		{
//			ForceContact *f = new ForceContact ( name, collisionWorld, stiff, damp, frict, thresh );
//			//m_forces[f->getName()] = f;
//			m_forces.push_back(f);
//			return f;
//		}
//		else
//			return NULL;
//	}
//
//	ForceContact * Model3::addForceContact ( string const& name, double stiff, double damp, double frict, double thresh)
//	{
//		bool nameAlreadyUsed = searchForcesForName(name);
//		if (!nameAlreadyUsed)
//		//if (m_forces.find(name) == m_forces.end())
//		{
//			ForceContact *f = new ForceContact ( name, stiff, damp, frict, thresh );
//			//m_forces[f->getName()] = f;
//			m_forces.push_back(f);
//			return f;
//		}
//		else
//			return NULL;
//	}
//#endif
}
