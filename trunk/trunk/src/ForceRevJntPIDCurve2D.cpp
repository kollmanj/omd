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
#include "ForceRevJntPIDCurve2D.h"
#include "JointRevolute.h"
#include <iostream>
namespace OMD
{
ForceRevJntPIDCurve2D::ForceRevJntPIDCurve2D(string const& name,
                            JointRevolute * jnt,double p, double i, double d, Curve2D *curve)
                            :m_joint(jnt),Force ( name ), m_p(p), m_i(i), m_d(d)
{
	m_curves.push_back(curve);
    m_t0 = 0;
    m_e0 = 0;
}

ForceRevJntPIDCurve2D::ForceRevJntPIDCurve2D(string const& name,
                            JointRevolute * jnt,double p, double i, double d, std::vector<Curve2D *> curves)
                            :m_joint(jnt),Force ( name ), m_p(p), m_i(i), m_d(d), m_curves(curves)
{
    m_t0 = 0;
    m_e0 = 0;
}

ForceRevJntPIDCurve2D::~ForceRevJntPIDCurve2D(void)
{
}

void ForceRevJntPIDCurve2D::apply(double t)
{
    double target = 0;

	std::vector<Curve2D *>::iterator it;
	for ( it=m_curves.begin() ; it < m_curves.end(); ++it )
	{
		target += (*it)->getDependent(t);
	}

    double error = target - m_joint->m_q;

    double dt = t-m_t0;
    double de = error-m_e0;
    double torque;
    if ( dt != 0 )
    {
      torque = m_p*error + m_d*de/dt + m_i*(m_e0+error*dt);
    }
    else
    {
      torque =  m_p*error + m_i*(m_e0+error*dt);
    }

    Vect3 torque_on_parent = m_joint->m_axis_pk * -torque;
    Vect3 torque_on_child = m_joint->m_axis_k*torque;
    // can apply torque at cg becuase it dosn't matter for a rigid body
	m_joint->m_parent->torqueAccum(torque_on_parent,true);
	m_joint->m_child->torqueAccum(torque_on_child,true);

    m_t0=t;
    m_e0=error;
//    std::cout << "Error: " << error << std::endl;
}


}
