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
#ifndef OMDOMDTREE_H
#define OMDOMDTREE_H
#include "Branch.h"
#include <vector>
#include "BodyRigid.h"
namespace OMD
{

	/**
		@author John Kollman
	*/
	class Tree
	{
		public:
			Tree();

			~Tree();
			///
			/// Add a Branch to this Tree
			///
			/// @param[in] branch branch to be added
			///
			void AddBranch(Branch* branch);
			///
			/// Add a Joint to this Tree
         /// It will figure out where to put it, or if it cannot connect
			///
			/// @param[in] joint joint to add
			/// @return Noting
			///
			bool addJoint(Joint1DOF* joint);
			///
			/// Set The a root joint for the tree, i.e. one that is connected to inertial ref. frame
			/// will make a "root" branch
			///
			/// @param[in] joint root joint
			/// @return Nothing
			///
			void setRoot(Joint1DOF* joint);
			///
			/// Add a joint to the tree as child of parent
			///
			/// @param[in] parent joint which is the parent, should be in here already
			/// @param[in] child joint
			/// @return Nothing
			///
			void AddChild(Joint1DOF* parent, Joint1DOF *child);
			///
			/// Does Tree contain a particular joint?
			///
			/// @param[in] joint joint to search for
			/// @return true if it's in there false if not
			///
			bool hasJoint(Joint * joint);
			bool hasBody(BodyRigid* body);
			///
			/// Get the number of branches in tree
			///
			/// @return number of branches
			///
			unsigned int getNumberOfBranches();
			///
			/// Get the branch of tree
			///
			/// @return pointer to branch
			///
			Branch * GetBranch(unsigned int i = 0);
			///
			/// Do Kinematics
			///
			/// @return nothing
			///
			void kinematics( double t );
			void triangularize( double t );
			void ForwardSubstitute( double t );
			std::vector<double> getState( );
			unsigned int getStateSize();
			unsigned int setState( std::vector<double> state_vector );
			std::vector<double> getDot( );
			void setBodyAccels();
			///
			/// print the contents of the tree
			///
			void print();
		private:
			std::vector<Branch *> m_branches;
			///
			/// Vector of joints which are connected to inertial reference frame
			///
			///vector<Joint *> m_roots;
	};

}

#endif
