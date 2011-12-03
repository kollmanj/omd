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
#ifndef OMDBRANCH_H
#define OMDBRANCH_H

#include  <string.h>
#include <vector>
//#include "OMDFwd.h"
#include "Joint1DOF.h"
namespace OMD
{
using namespace std;
	/**
		@author John Kollman,,, <john@john-worklaptop>
	*/
	class Branch
	{
		public:

			Branch();
			///
			/// create a branch
			///
			/// @param[in] joint base joint
			/// @param[in] b is joint connected to ground?
			///
			Branch(Joint1DOF *joint, unsigned int i = 0);
			Branch(Branch *b0);

			~Branch();


			///
			/// Do Kinematics
			///
			/// @return nothing
			///
			void kinematics( double t );
			///
			/// Find Joint in the Branch
			///
			/// @param[in] joint pointer to joint
			/// @return -1 if NOT present otherwise number in consecutively linked joints starting w/ 0
			///
			int findJoint(Joint *joint);
			///
			/// Determine if Joint is in the Branch
			///
			/// @param[in] joint pointer to joint
			/// @return true yes false no
			///
			bool hasJoint(Joint *joint);
			bool hasBody(BodyRigid *body);
         ///
			/// Determine if Body is in the Branch
         /// TODO:  allow for the possibility that it is found twice and communicate that to user
			///
			/// @param[in] body pointer to body
			/// @return pointer to joint if it is there
			///
			///
			/// Add a joint to the end of a Branch
			///
			/// @param[in] joint pointer to joint
			/// @return Nothing
			///
			void addJoint(Joint1DOF *joint);
			///
			/// tell branch what level it is
			/// @param[in] level of the branch, 0:  connected to ground
			/// @return Nothing
			///
			void setLevel(unsigned int i = 0);
			///
			/// Set Level 0:  connected to ground
			///
			unsigned int getLevel();
			unsigned int Size();
			void triangularize(double t);
			void ForwardSubstitute( double t );

			std::vector<double > getState( );
			unsigned int getStateSize();
			unsigned int setState( std::vector<double> state_vector );
         ///
         /// Get derivitives
         /// @param[in] empty pointer to be filled
         /// @return number of dots
         ///
			std::vector<double > getDot( );
			Joint1DOF * getLastJoint();
			bool isLastBody(BodyRigid * body);
			void setBodyAccels();
			///
			/// print the contents of the branch
			///
			void print();
		private:
			///
			/// Vector of joint that are part of the branch
			///
			std::vector<Joint1DOF *> m_joints;
			///
			/// level of the branch, 0:  connected to ground
			///
			unsigned int m_level;

	};

}

#endif
