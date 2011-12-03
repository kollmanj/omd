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
#include "ForceTransJnt.h"
#include "JointTranslational.h"

namespace OMD
{
   ForceTransJnt::ForceTransJnt(string const& name,
                            JointTranslational & jnt,
                            double frc):m_joint(jnt),Force ( name ), m_force(frc)
{
}

ForceTransJnt::~ForceTransJnt(void)
{
}

	void ForceTransJnt::apply(double t)
	{
      Vect3 force_on_parent = m_joint.m_axis_pk * -m_force;
      Vect3 force_on_child = m_joint.m_axis_k*m_force;
      //m_joint.m_parent->ForceAccum(force_on_parent,m_joint.r_pk_j.cross(force_on_parent));
      //m_joint.m_child->ForceAccum(force_on_child,m_joint.m_r_j_k.cross(force_on_child));
	  m_joint.m_parent->forceAccum(force_on_parent,true,m_joint.r_pk_j);
	  m_joint.m_child->forceAccum(force_on_child,true,m_joint.m_r_j_k);
   }

   void ForceTransJnt::SetForce(double frc)
   {
      m_force = frc;
   }
}
