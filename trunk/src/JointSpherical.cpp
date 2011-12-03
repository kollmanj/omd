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
#include "JointSpherical.h"

namespace OMD
{

	JointSpherical::JointSpherical ( string const& name, BodyRigid* parent, Vect3 parent2joint, BodyRigid* child, Vect3 joint2child, double q0, double u0 ) : Joint ( name, parent, parent2joint, child, joint2child)
	{

	}
	JointSpherical::JointSpherical ( string const& name, BodyRigid* parent, vector<double> parent2joint, BodyRigid* child, vector<double> joint2child,double q0, double u0 ): Joint(name, parent, parent2joint, child, joint2child)
	{
	}

	JointSpherical::~JointSpherical()
	{
	}

	MatNxN JointSpherical::getJacobian( int ParentOrChild )
	{
		return  getJacobianS3(ParentOrChild);
	}

	MatNxN JointSpherical::getJacobianModified(int ParentOrChild)
	{
		return getJacobianModS3(ParentOrChild);
	}

	vector<double> JointSpherical::getViolation()
	{
		vector<double> out;
		MatNxN s_3 = getConstraintViolationS3();
		out.push_back(s_3(0,0));
		out.push_back(s_3(1,0));
		out.push_back(s_3(2,0));
		return out;
	}

	VectN JointSpherical::getGamaPound()
	{
		return getGamaPoundS3();
	}
}
