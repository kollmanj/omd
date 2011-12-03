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
#ifndef OMDJOINTSPHERICAL_H
#define OMDJOINTSPHERICAL_H

#include "Joint.h"

namespace OMD
{

	///
	///	@author John Kollman
	///
	class JointSpherical : public Joint
	{
		public:
			JointSpherical ( string const& name, BodyRigid* parent, Vect3 parent2joint, BodyRigid* child, Vect3 joint2child,double q0=0, double u0=0 );
			JointSpherical ( string const& name, BodyRigid* parent, vector<double> parent2joint, BodyRigid* child, vector<double> joint2child,double q0=0, double u0=0 );

			~JointSpherical();

    ///
    /// Modified Constraint Jacobian
    /// page 299 Nikravesh
    virtual MatNxN getJacobianModified( int ParentOrChild);
    virtual MatNxN getJacobian( int ParentOrChild );
    virtual VectN getGamaPound();
    virtual vector<double> getViolation();
   private:
      //friend class ForceRevJnt;
      //friend class ForceRevJntPIDCurve2D;
      //friend class ForceRevJntSpringDamp;


	};

}

#endif
