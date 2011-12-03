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
#include "JointUniversal.h"

namespace OMD
{

	JointUniversal::JointUniversal ( string const& name, BodyRigid* parent, Vect3 parent2joint, Vect3 pAxis, BodyRigid* child, Vect3 joint2child,Vect3 cAxis) :
		JointAxis ( name, parent, parent2joint, pAxis, child, joint2child, cAxis)
	{

	}

	JointUniversal::JointUniversal ( std::string const& name, BodyRigid* parent, vector<double> parent2joint, vector<double> pAxis, BodyRigid* child, vector<double> joint2child, vector<double> cAxis ):
		JointAxis ( name, parent, parent2joint, pAxis, child, joint2child, cAxis)
	{
	}

	JointUniversal::~JointUniversal()
	{
	}

	MatNxN JointUniversal::getJacobian( int ParentOrChild )
	{
		MatNxN s3 = getJacobianS3(ParentOrChild);
		MatNxN n11 = getJacobianN11a(ParentOrChild);

		// vertical concatinate
		MatNxN out;
		out = concatV(s3, n11);
		//std::cout << out << std::endl; //checks out
		return out;
	}

	MatNxN JointUniversal::getJacobianModified(int ParentOrChild)
	{
		MatNxN s3 = getJacobianModS3(ParentOrChild);
		MatNxN n11 = getJacobianModN11a(ParentOrChild);

		// vertical concatinate
		MatNxN out;
		out = concatV(s3, n11);
		//std::cout << out << std::endl;
		return out;
	}

	vector<double> JointUniversal::getViolation()
	{
		vector<double> out;
		MatNxN s3 = getConstraintViolationS3();
		MatNxN n11 = getConstraintViolationN11a();

		out.push_back(s3(0,0));
		out.push_back(s3(1,0));
		out.push_back(s3(2,0));
		out.push_back(n11(0,0));

		return out;
	}

	VectN JointUniversal::getGamaPound()
	{
		VectN s3 = getGamaPoundS3();
		VectN n11 = getGamaPoundN11a();

		// vertical concatinate
		VectN out;
		out = concatV(s3,n11);
		return out;
	}
}
