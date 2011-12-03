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
#ifndef OMDOMDJOINTTRANS_H
#define OMDOMDJOINTTRANS_H

#include "Joint1DOF.h"

namespace OMD {

/**
	@author John Kollman,,, <john@john-worklaptop>
*/
//using namespace OMD;
class JointTranslational : public Joint1DOF
{
public:
	///
	/// Construct a translational joint without Eigen for use in swig
	///
	/// @param[in] name	name of joint
	/// @param[in] parent parent body
	/// @param[in] parent2joint is vector from parent cg to joint
	/// @param[in] child child body
	/// @param[in] joint2child is the vector from joint to childs
	/// @param[in] axis direction DOF in parent
	/// @return The status of the call
	///
    JointTranslational(string const &name, BodyRigid * parent,
		vector<double> const &parent2joint,
		BodyRigid *child,
		vector<double> const &joint2child,
		vector<double> const &axis,
		double q0 = 0., double u0 = 0.);
	///
	/// Construct a translational joint
	///
	/// @param[in] name	name of joint
	/// @param[in] parent parent body
	/// @param[in] parent2joint is vector from parent cg to joint
	/// @param[in] child child body
	/// @param[in] joint2child is the vector from joint to childs
	/// @param[in] axis direction DOF in parent
	/// @return The status of the call
	///
    JointTranslational(string const &name, BodyRigid * parent,
		Vect3 parent2joint,
		BodyRigid *child,
		Vect3 joint2child,
		Vect3 axis,
		double q0 = 0., double u0 = 0.);


    virtual void kinematicsPosition( double t );
    virtual Vect3 calcChildVel( double t );
    virtual Vect3 calcChildAngVel( double t );
    virtual Vect3 KinematicsAccelLin( double t );
    virtual Vect3 KinematicsAccelAng( double t );

    virtual MatNxN getJacobianModified( int ParentOrChild );
    virtual MatNxN getJacobian( int ParentOrChild );
    virtual VectN getGamaPound();
    virtual vector<double> getViolation();	
	///
	/// sorry for the terrible name but it's physical significance is ?
	/// this is equestion 41 page 51 of Anderson
	///
	virtual Vect3 Nv_r();
	/// this is equestion 40 page 51 of Anderson
	virtual Vect3 Nw_r();
    ~JointTranslational();
private:
//	virtual void Setup();
	Vect3 m_gamma0;
//	Vect3 m_axis_pk;
	Vect3 m_v1;
	Vect3 m_v2;
   private:

	   ///TODO: put these back in
      //friend class ForceTransJnt;
      //friend class ForceTransJntSpringDamp;
};

}

#endif
