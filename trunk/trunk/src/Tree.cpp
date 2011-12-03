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
#include "Tree.h"
#include <iostream>

namespace OMD
{

	Tree::Tree()
	{
	}
	unsigned int Tree::getStateSize()
	{
		unsigned int statesize = 0;
		for ( unsigned int i = 0; i<m_branches.size(); ++i )
		{
			statesize += m_branches[i]->getStateSize();
		}
		return statesize;
	}

	Tree::~Tree()
	{
//		delete root_branch;
/// If you have a crash in here some model with nothing in it is being delete it
		/// really can't have a empty model TODO: Fix that!
		for ( unsigned int i = 0; i<m_branches.size(); ++i )
		{
			delete m_branches[i];
		}
	}

	void Tree::kinematics( double t )
	{
		for ( unsigned int i = 0; i<m_branches.size(); ++i )
		{
			m_branches[i]->kinematics( t );
		}
	}


	void Tree::setRoot(Joint1DOF *joint)
	{
//		m_root = joint;
		Branch *root_branch;
		// make root branch
		root_branch = new Branch(joint, 0);
		m_branches.push_back ( root_branch );
	}

	void Tree::AddBranch(Branch* branch)
	{
		m_branches.push_back ( branch );
	}

   bool Tree::addJoint(Joint1DOF* joint)
   {

      // first get the parent body of the joint to determine if that
      // body is the child body in a joint in any of the branches of the tree
      BodyRigid *parent = joint->getParent();
//
 //     Joint *joint_where_parent_is_child = NULL;
//
      std::vector< Branch *>::iterator it;
		for (it = m_branches.begin(); it < m_branches.end(); ++it )
		{
         // *it , in this case, is a branch
			bool branch_has_body = (*it)->hasBody(parent);
			bool body_is_last = (*it)->isLastBody(parent);
			if ( branch_has_body )
			{
				if ( body_is_last )
				{
					(*it)->addJoint( joint );
				}
				else
				{
					// make another branch
					Branch *root_branch;
					// make root branch
					root_branch = new Branch(joint, 1);
					m_branches.push_back ( root_branch );
				}
				return true;
			}

		}
		return false;

   }

	void Tree::AddChild(Joint1DOF* parent, Joint1DOF *child)
	{

		// look through multimap to find all the branches in which the parent exists
		for ( unsigned int i = 0; i<m_branches.size(); ++i )
		{
			int index = m_branches[i]->findJoint(parent);
			if (index != -1)
			{
				// if the parent is at the end of the branch just add the child to the end
				if (index == m_branches[i]->Size()-1)
				{
					m_branches[i]->addJoint(child);
				}
				else  // if the parent is some place other then the end, need a new branch
				{
					Branch *branch;
					branch = new Branch(child);
					m_branches.push_back ( branch );

					/*
					Branch *branch;
					branch = new Branch(m_branches[i]);
					branch->addJoint(child);
					m_branches.push_back ( branch );
					*/


				}
			}
		}
		// make a branch
/*
		Branch *parentbranch;

		Branch *childbranch;
		m_branches.insert ( pair<Branch *, Branch *> (parentbranch, childbranch) );*/
	}
	void Tree::triangularize( double t )
	{
		//triangularize from bottom up
		std::vector< Branch *>::reverse_iterator rit;
		for (rit = m_branches.rbegin(); rit < m_branches.rend(); ++rit )
		{
			(*rit)->triangularize(t);
		}
	}

	void Tree::ForwardSubstitute( double t )
	{
		//forward substitute top down
		std::vector< Branch *>::iterator it;
		for (it = m_branches.begin(); it < m_branches.end(); ++it )
		{
			(*it)->ForwardSubstitute(t);
		}
	}

	void Tree::setBodyAccels()
	{
		for ( unsigned int i = 0; i<m_branches.size(); ++i )
		{
			m_branches[i]->setBodyAccels();
		}
	}

	unsigned int Tree::setState( std::vector<double> state_vector )
	{

      unsigned int state_size = 0;
		std::vector< Branch *>::iterator it;
		for (it = m_branches.begin(); it < m_branches.end(); ++it )
		{
         // get state size of the joint
         unsigned int branch_state_size = (*it)->getStateSize();
         std::vector<double> branch_state_vector;
         for (int i = state_size; i< branch_state_size + state_size; i++)
         {
            branch_state_vector.push_back( state_vector[i] );
         }
         (*it)->setState( branch_state_vector );

         state_size += branch_state_size;
		}
      return state_size;

	}

	std::vector<double> Tree::getState( )
	{
		unsigned int state_size = 0;
      std::vector<double> state_vector;
		std::vector< Branch *>::iterator it;
		for (it = m_branches.begin(); it < m_branches.end(); ++it )
		{
         std::vector<double> state_vector_branch = (*it)->getState( );
         state_size += state_vector_branch.size( );

         std::vector<double>::iterator it2;
         for (it2 = state_vector_branch.begin(); it2 < state_vector_branch.end(); ++it2)
         {
            state_vector.push_back(*it2);
         }

		}
      return state_vector;
	}

	std::vector<double> Tree::getDot( )
	{

      std::vector<double> dot_vector;
		std::vector< Branch *>::iterator it;
		for (it = m_branches.begin(); it < m_branches.end(); ++it )
		{
         std::vector<double> dot_vector_branch = (*it)->getDot( );

         std::vector<double>::iterator it2;
         for (it2 = dot_vector_branch.begin(); it2 < dot_vector_branch.end(); ++it2)
         {
            dot_vector.push_back(*it2);
         }

		}
      return dot_vector;

	}


	bool Tree::hasJoint(Joint * joint)
	{
		for ( unsigned int i = 0; i<m_branches.size(); ++i )
		{
			if (m_branches[i]->hasJoint(joint))
			{
				return true;
			}
		}
		return false;
	}

	bool Tree::hasBody(BodyRigid * body)
	{
		for ( unsigned int i = 0; i<m_branches.size(); ++i )
		{
			if (m_branches[i]->hasBody(body))
			{
				return true;
			}
		}
		return false;
	}

	unsigned int Tree::getNumberOfBranches()
	{
		return m_branches.size();
	}

	Branch * Tree::GetBranch(unsigned int i )
	{
		return m_branches[i];
	}

	void Tree::print()
	{
		for ( unsigned int i = 0; i<m_branches.size(); ++i )
		{
			cout << "Branch Level: " << m_branches[i]->getLevel() << " ";
			cout << "Joints of Branch " << i << " are: " << endl;
			m_branches[i]->print();
		}
	}

}
